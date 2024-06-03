#include "display.h"
#include "hardware/clocks.h"

#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include "hardware/pwm.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define PWM_PIN 5
#define BAUDRATE 9600
#define USE_UART

#define UART_RX_PIN 17
#define UART_TX_PIN 16

/* GPIO21: CLK_GPOUT0 → 26 MHz RF reference for u-blox (clk_sys/4 @ 104 MHz SYS). */
#define GPS_REFCLK_OUT_PIN 21

#include "gpsdo_control.h"
#include "gpsdo_tune.h"
#include "ubx_clean.h"

#include "pico/error.h"

bool g_ubx_uart_hex_dump = false;
bool g_json_telemetry_print = true;

bool ssd1306_ok = false;

#if PICO_USB_BOOT_REENUM
/* TinyUSB + RP2040 DCD: toggling the pull-up makes the host see disconnect/reconnect (full re-enumeration). */
static void usb_host_force_reenumerate(void) {
	if (!tud_inited()) {
		return;
	}
	tud_disconnect();
	sleep_ms(100);
	tud_connect();
	sleep_ms(50);
}
#endif

//#define ALT_CPU

// for 26Mhz
#ifdef ALT_CPU
#define CLOCK_DIV 4.0
#else
#define CLOCK_DIV 1.0
#endif

void display() {
	char text[255];

	memset(buf, 0, SSD1306_BUF_LEN);

	sprintf(text, "clkB %d ", abs(current_gps_data.clkB));
	WriteString(buf, 0, 10, text);

	sprintf(text, "qErr %d ", abs(current_gps_data.qErr));
	WriteString(buf, 0, 18, text);

	sprintf(text, "fAcc %d ", current_gps_data.fAcc);
	WriteString(buf, 0, 26, text);

	sprintf(text, "dac %d ", current_gps_data.dac);
	WriteString(buf, 0, 34, text);

	sprintf(text, "sats %d x %d ", current_gps_data.satellites_visible, current_gps_data.fix);
	WriteString(buf, 0, 42, text);

	sprintf(text, "qAge %.1f  ", (double)current_gps_data.qAge);
	WriteString(buf, 0, 50, text);

	//memset(buf, 0xFF, SSD1306_BUF_LEN);

	//DrawLine(buf, 0, 20, 128, 20, true);

	//DrawLine(buf, 0, 30, 128, 30, true);

	//DrawLine(buf, 0, 0, 128, 64, true);
	//DrawLine(buf, 0, 40, 128, 40, true);
	render(buf);
}

uint fullScale = 8192 * 4 / CLOCK_DIV;

static uint g_pwm_slice_for_manual;

/* Fractional DAC counts not yet applied (PWM is integer steps). */
static float g_dac_frac_accum;
/* Per-DAC learned local slope score: sign(score) is slope sign; magnitude is confidence. */
static int8_t g_qerr_slope_score_by_dac[32768 + 1];
static int32_t g_edge_last_dac;
static int32_t g_edge_last_qerr_ps;
static bool g_edge_has_prev;
static bool g_edge_eval_pending;
static int32_t g_edge_eval_countdown;
static int32_t g_edge_eval_window_samples;
static int32_t g_edge_eval_from_dac;
static int32_t g_edge_eval_from_qerr_ps;
static int32_t g_edge_eval_step;
static int32_t g_qerr_hist_ps[64];
static int32_t g_qerr_hist_pos;
static int32_t g_qerr_hist_count;
static bool g_seen_nonzero_qerr;
static void fine_edge_reset(void);

static void setDAC_hw(uint slice, uint32_t dac) {
	if (dac > fullScale) {
		dac = fullScale;
	}
	pwm_set_chan_level(slice, PWM_PIN, dac);
	current_gps_data.dac = dac;
	current_gps_data.adjustDac = false;
}

void setDAC(uint slice, uint32_t dac) {
	g_dac_frac_accum = 0.0f;
	fine_edge_reset();
	setDAC_hw(slice, dac);
}

static void apply_manual_dac(uint32_t dac) {
	setDAC(g_pwm_slice_for_manual, dac);
}

float deltaToDac(float delta_ns) {  // 87ns = 1000 dac
	int32_t a = -delta_ns * 12.0481927711 / CLOCK_DIV;
	if (a > 4000 / CLOCK_DIV) { a = 4000 / CLOCK_DIV; }
	if (a < -4000 / CLOCK_DIV) { a = -4000 / CLOCK_DIV; }

	return (float)a;
}

static void dac_apply_delta(uint slice_num, float delta_counts) {
	g_dac_frac_accum += delta_counts;
	int32_t step = (int32_t)truncf(g_dac_frac_accum);
	if (step == 0) {
		return;
	}
	g_dac_frac_accum -= (float)step;

	int32_t old_dac = (int32_t)current_gps_data.dac;
	int64_t nd = (int64_t)old_dac + (int64_t)step;
	if (nd < 0) {
		nd = 0;
	}
	if (nd > (int64_t)fullScale) {
		nd = fullScale;
	}
	int32_t new_dac = (int32_t)nd;
	int32_t applied_delta = new_dac - old_dac;
	if (applied_delta != step) {
		g_dac_frac_accum += (float)(step - applied_delta);
	}
	setDAC_hw(slice_num, (uint32_t)new_dac);
}

static int32_t sign_i32(int32_t x) {
	if (x > 0) {
		return 1;
	}
	if (x < 0) {
		return -1;
	}
	return 0;
}

static int8_t clamp_i8(int32_t x) {
	if (x > 127) {
		return 127;
	}
	if (x < -127) {
		return -127;
	}
	return (int8_t)x;
}

static int32_t abs_i32(int32_t x) {
	return x >= 0 ? x : -x;
}

static void fine_edge_update_score_at(int32_t dac, int32_t delta) {
	if (dac < 0 || dac > 32768 || delta == 0) {
		return;
	}
	int32_t s = (int32_t)g_qerr_slope_score_by_dac[dac];
	/* Slow forgetting so the map can adapt over days. */
	if (s > 0) {
		s--;
	} else if (s < 0) {
		s++;
	}
	s += delta;
	g_qerr_slope_score_by_dac[dac] = clamp_i8(s);
}

static void fine_edge_reset(void) {
	memset(g_qerr_slope_score_by_dac, 0, sizeof(g_qerr_slope_score_by_dac));
	g_edge_last_dac = 0;
	g_edge_last_qerr_ps = 0;
	g_edge_has_prev = false;
	g_edge_eval_pending = false;
	g_edge_eval_countdown = 0;
	g_edge_eval_window_samples = 0;
	g_edge_eval_from_dac = 0;
	g_edge_eval_from_qerr_ps = 0;
	g_edge_eval_step = 0;
	g_qerr_hist_pos = 0;
	g_qerr_hist_count = 0;
}

static void qerr_hist_push(int32_t qerr_ps) {
	g_qerr_hist_ps[g_qerr_hist_pos] = qerr_ps;
	g_qerr_hist_pos = (g_qerr_hist_pos + 1) % 64;
	if (g_qerr_hist_count < 64) {
		g_qerr_hist_count++;
	}
}

static bool qerr_hist_get_n_ago(int32_t n, int32_t* out_qerr_ps) {
	if (n < 0 || n >= g_qerr_hist_count || out_qerr_ps == NULL) {
		return false;
	}
	int32_t idx = g_qerr_hist_pos - 1 - n;
	while (idx < 0) {
		idx += 64;
	}
	*out_qerr_ps = g_qerr_hist_ps[idx];
	return true;
}

static void fine_edge_arm_eval(int32_t dac_before, int32_t qerr_before_ps, int32_t applied_step, const gpsdo_tune_t* tune) {
	int32_t n = tune->fine_edge_eval_samples;
	if (n < 1) {
		n = 1;
	}
	g_edge_eval_pending = true;
	g_edge_eval_countdown = n;
	g_edge_eval_window_samples = n;
	g_edge_eval_from_dac = dac_before;
	g_edge_eval_from_qerr_ps = qerr_before_ps;
	g_edge_eval_step = applied_step;
}

static void fine_edge_track_observation(int32_t dac, int32_t qerr_ps) {
	qerr_hist_push(qerr_ps);

	if (g_edge_eval_pending) {
		g_edge_eval_countdown--;
		if (g_edge_eval_countdown <= 0) {
			int32_t ddac = g_edge_eval_step;
			int32_t q_n_ago = g_edge_eval_from_qerr_ps;
			/* Explicit sliding-window evaluation: qErr(now) - qErr(now-N samples). */
			if (!qerr_hist_get_n_ago(g_edge_eval_window_samples, &q_n_ago)) {
				q_n_ago = g_edge_eval_from_qerr_ps;
			}
			int32_t dq = qerr_ps - q_n_ago;
			if (ddac != 0 && dq != 0) {
				int64_t prod = (int64_t)ddac * (int64_t)dq;
				int32_t slope_sign = (prod > 0) ? 1 : -1;
				fine_edge_update_score_at(dac, slope_sign * 4);
				fine_edge_update_score_at(g_edge_eval_from_dac, slope_sign * 4);

				/* If error kept the same sign and grew over the eval window, penalize this slope. */
				int32_t prev_abs = abs_i32(g_edge_eval_from_qerr_ps);
				int32_t now_abs = abs_i32(qerr_ps);
				if (sign_i32(g_edge_eval_from_qerr_ps) == sign_i32(qerr_ps) && now_abs > prev_abs + 20) {
					/* Strong inversion on wrong-way behavior to recover quickly. */
					fine_edge_update_score_at(dac, -slope_sign * 16);
					fine_edge_update_score_at(g_edge_eval_from_dac, -slope_sign * 16);
				}
			}
			g_edge_eval_pending = false;
			g_edge_eval_countdown = 0;
			g_edge_eval_window_samples = 0;
			g_edge_eval_step = 0;
		}
	}

	g_edge_last_dac = dac;
	g_edge_last_qerr_ps = qerr_ps;
	g_edge_has_prev = true;
}

/* Returns -1/0/+1 DAC step. Learns local qErr slope from observed DAC transitions. */
static int32_t fine_edge_pick_step(const gps_data* data, const gpsdo_tune_t* tune) {
	(void)tune;

	int32_t dac = (int32_t)data->dac;
	int32_t slope = 0;
	if (dac >= 0 && dac <= 32768) {
		slope = sign_i32((int32_t)g_qerr_slope_score_by_dac[dac]);
	}
	if (slope == 0 && dac > 0 && dac <= 32768) {
		slope = sign_i32((int32_t)g_qerr_slope_score_by_dac[dac - 1]);
	}
	if (slope == 0 && dac >= 0 && dac < 32768) {
		slope = sign_i32((int32_t)g_qerr_slope_score_by_dac[dac + 1]);
	}
	if (slope == 0) {
		/* Fallback for startup until slope map is learned. */
		slope = 1;
	}

	/* Reduce qErr magnitude: step direction is opposite sign(qErr) under learned slope. */
	int32_t qsign = sign_i32(data->qErr);
	int32_t desired = -qsign * slope;
	if (desired > 1) {
		desired = 1;
	}
	if (desired < -1) {
		desired = -1;
	}
	if (dac <= 0 && desired < 0) {
		desired = 0;
	}
	if (dac >= (int32_t)fullScale && desired > 0) {
		desired = 0;
	}
	return desired;
}

/* USB CDC: one line per command (see gpsdo_tune_try_cli_line). */
static void poll_usb_tune_cli(void) {
	static char line_buf[96];
	static size_t line_len;

	for (;;) {
		int c = getchar_timeout_us(0);
		if (c == PICO_ERROR_TIMEOUT) {
			break;
		}
		if (c == '\r' || c == '\n') {
			if (line_len > 0) {
				line_buf[line_len] = '\0';
				if (!gpsdo_tune_try_cli_line(line_buf)) {
					if (strncmp(line_buf, "ubxhex ", 7) == 0) {
						char ch = line_buf[7];
						if (ch == '0' || ch == '1') {
							g_ubx_uart_hex_dump = (ch == '1');
							printf("[t] ubxhex %d\n", g_ubx_uart_hex_dump ? 1 : 0);
							stdio_flush();
						}
					} else if (strncmp(line_buf, "json ", 5) == 0) {
						char ch = line_buf[5];
						if (ch == '0' || ch == '1') {
							g_json_telemetry_print = (ch == '1');
							printf("[t] json %d\n", g_json_telemetry_print ? 1 : 0);
							stdio_flush();
						}
					} else if (isalpha((unsigned char)line_buf[0])) {
						printf("[t] ? help\n");
						stdio_flush();
					}
				}
				line_len = 0;
			}
		} else if (line_len < sizeof(line_buf) - 1) {
			line_buf[line_len++] = (char)c;
		}
	}
}

int main() {
	bool finelock = false;
	bool prev_finelock = false;
	bool prev_edge_mode = false;
	static float finelock_enter_accum_s;

	/* USB stdio must be first: printf before stdio_init_all() is silent. */
	stdio_init_all();
	printf("\nfw\n");
	stdio_flush();

	/*
	 * OCXO-servoed PWM (DAC) + 26 MHz GPS reference — right after first log so
	 * the u-blox sees a clock within a few ms of boot (only a short gap after reset).
	 */
	gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
	uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
	g_pwm_slice_for_manual = slice_num;
	gpsdo_tune_set_manual_dac_callback(apply_manual_dac);
	pwm_set_wrap(slice_num, fullScale);
	pwm_set_enabled(slice_num, true);
	/* 104 MHz clk_sys / 4 → 26 MHz on GPS_REFCLK_OUT_PIN */
	clock_gpio_init(GPS_REFCLK_OUT_PIN, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 4 / CLOCK_DIV);
	setDAC(slice_num, 17922 / CLOCK_DIV);
	printf("ref %u 26MHz\n", GPS_REFCLK_OUT_PIN);
	stdio_flush();

	const uint LED_PIN = PICO_DEFAULT_LED_PIN;

	printf("usb 500ms\n");
	stdio_flush();
	sleep_ms(500);

#if PICO_USB_BOOT_REENUM
	/* Do not printf() while disconnected — CDC is down. */
	usb_host_force_reenumerate();
	printf("usb reenum\n");
	stdio_flush();
#endif

	SSD1306_init();
	SSD1306_scroll(false);

	display();

	bool reset = true;

	if (reset) {
		while (true) {
			if (ubx_init()) {
				printf("ubx ok\n");
				stdio_flush();
				break;
			}
			printf("ubx retry 1s\n");
			stdio_flush();
			sleep_ms(1000);
		}
	} else {
		uart_init_ubx(38400);
	}
	gpsdo_tune_init_defaults();
	gpsdo_control_reset_state();
	fine_edge_reset();
	printf("[t] help|ver|mode|dac|…\n");
	gpsdo_tune_print_version_banner();
	stdio_flush();
	//	setDAC(slice_num, 18922);  // == 83

	//	setDAC(slice_num, 16922);  // == - 87
	current_gps_data.adjustDac = false;
	current_gps_data.newQerr = false;
	current_gps_data.newBias = false;
	g_seen_nonzero_qerr = false;

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	gpio_put(LED_PIN, 1);
	uint32_t eval_duration = 5;

	uint32_t loop_count = 0;

	while (true) {
		/* UBX RX only via UART0 IRQ (ubx_poll in irq handler). Do not call ubx_poll() here:
		 * ubx_handle_byte() uses static parser state; IRQ + main-loop polling races and drops ACKs. */
		poll_usb_tune_cli();
		/* Heartbeat: early boot logs are lost before the host opens the CDC tty after flash. */
		loop_count++;
		if (loop_count % 30 == 0) {
			printf("[hb] %u\n", (unsigned)loop_count);
			stdio_flush();
		}

		sleep_ms(100);
		if (current_gps_data.newBias) {
			gpsdo_control_on_new_bias(&current_gps_data);
		}
		finelock = gpsdo_control_update_finelock(&current_gps_data, &gpsdo_tune, finelock, &finelock_enter_accum_s);
		const bool edge_mode_now = (gpsdo_tune.fine_mode == 1);
		/* Reset edge learner when leaving edge mode or dropping out of finelock into coarse. */
		if ((prev_edge_mode && !edge_mode_now) || (prev_finelock && !finelock)) {
			fine_edge_reset();
		}
		prev_edge_mode = edge_mode_now;
		prev_finelock = finelock;
		display();
		//current_gps_data.ticks++;
		/* One line per loop when TIM-TP and/or NAV-CLOCK updated. Always print the full struct
		 * snapshot so the host never sees long clkB gaps when only qErr refreshed this cycle. */
		if ((current_gps_data.newQerr || current_gps_data.newBias) && g_json_telemetry_print) {
			unsigned lp;
			if (gpsdo_run_mode == GPSDO_MODE_FREE) {
				lp = (unsigned)GPSDO_LOOP_INACTIVE;
			} else if (finelock) {
				lp = (unsigned)GPSDO_LOOP_FINE;
			} else {
				lp = (unsigned)GPSDO_LOOP_COARSE;
			}
			unsigned tr = gpsdo_control_track_flags(&current_gps_data, &gpsdo_tune);
			if (!g_seen_nonzero_qerr) {
				tr |= 8u;
			}
			printf(
					"{lp:%u,tr:%u,iT:%u,tMS:%u,tSub:%u,q:%d,cB:%d,cDb:%.3f,id:%.3f,cD:%d,tA:%u,fA:%u,dac:%u,sv:%u,tk:%u,fx:%u}\n",
					lp, tr, current_gps_data.iTOW, current_gps_data.towMS, current_gps_data.towSubMS,
					current_gps_data.qErr, current_gps_data.clkB, (double)current_gps_data.clkD_bias,
					(double)gpsdo_control_intended_drift(), current_gps_data.clkD,
					current_gps_data.tAcc, current_gps_data.fAcc, current_gps_data.dac,
					(unsigned)current_gps_data.satellites_visible, current_gps_data.ticks,
					(unsigned)current_gps_data.fix);
			stdio_flush();
		}

		if (current_gps_data.newQerr) {
			current_gps_data.newQerr = false;
			if (current_gps_data.qErr != 0) {
				g_seen_nonzero_qerr = true;
			}
			fine_edge_track_observation((int32_t)current_gps_data.dac, current_gps_data.qErr);

			if (gpsdo_run_mode == GPSDO_MODE_SERVO && finelock) {
				if (gpsdo_tune.fine_mode == 1) {
					if (!g_edge_eval_pending) {
						int32_t old_dac = (int32_t)current_gps_data.dac;
						int32_t step = fine_edge_pick_step(&current_gps_data, &gpsdo_tune);
						if (step != 0) {
							int32_t nd = old_dac + step;
							if (nd < 0) {
								nd = 0;
							}
							if (nd > (int32_t)fullScale) {
								nd = (int32_t)fullScale;
							}
							int32_t applied = nd - old_dac;
							if (applied != 0) {
								setDAC_hw(slice_num, (uint32_t)nd);
								fine_edge_arm_eval(old_dac, current_gps_data.qErr, applied, &gpsdo_tune);
							}
						}
					}
				} else {
					float u_ns = gpsdo_control_fine_u_ns(&current_gps_data, &gpsdo_tune);
					dac_apply_delta(slice_num, deltaToDac(u_ns));
				}
			}
		}

		/* Coarse: P on clkD_bias when clkB steps (adjustDac). */
		if (gpsdo_run_mode == GPSDO_MODE_SERVO && !finelock && current_gps_data.adjustDac) {
			bool facc_ok = (gpsdo_tune.facc_max_gate <= 0
					|| current_gps_data.fAcc <= (uint32_t)gpsdo_tune.facc_max_gate);
			if (facc_ok) {
				dac_apply_delta(slice_num, deltaToDac(gpsdo_control_coarse_u_ns(&current_gps_data, &gpsdo_tune)));
			}
		}

		if (current_gps_data.newBias) {
			current_gps_data.newBias = false;
		}

		//sprintf(text, "asd %u ", current_gps_data.dac_age);
		//WriteString(text, 25, 25, text);
		//render(buf);
	}
}
