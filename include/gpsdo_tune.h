#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Runtime-tunable GPSDO servo (USB CDC one line per command, ends with \\n).
 * Coarse: NAV-CLOCK P on clkD_bias when not finelock (only when clkB steps — adjustDac).
 * Fine: TIM-TP qErr P-only (kp) when finelock (|clkB| inside hysteresis band).
 *
 * JSON telemetry field `lp` (uint, loop stage) — stable host/UI contract:
 *   0 = coarse (servo, not finelock)
 *   1 = fine   (servo, finelock)
 *   2 = inactive (mode free — DAC not driven by servo)
 *
 * JSON `tr` (uint bitmask) — GNSS / servo health (non-zero = issue):
 *   +1: no fix (fixType == 0)
 *   +2: satellites_visible < min_sv (when min_sv > 0)
 *   +4: fAcc > facc_max_gate (coarse DAC skipped when gate enabled)
 *   +8: TIM-TP qErr never observed non-zero since boot (edge/fine input unhealthy)
 */
typedef enum {
	GPSDO_LOOP_COARSE = 0,
	GPSDO_LOOP_FINE = 1,
	GPSDO_LOOP_INACTIVE = 2,
} gpsdo_loop_stage_t;

typedef struct {
	/* Fine loop (TIM-TP qErr in ps): u_ns = kp_qerr * e_ns, e_ns=qErr*1e-3. */
	float kp_qerr;
	/* Coarse loop: P gain on clkD_bias (NAV-CLOCK, when clkB steps). */
	float kp_coarse;
	/* Skip coarse DAC adjust when fAcc > this (ps/s). Set 0 to disable gate. */
	int32_t facc_max_gate;
	/* If >0: tr bit1 when satellites_visible < min_sv. 0 = disable SV check. */
	int32_t min_sv;
	/* Hysteresis on |clkB| (ns): exit finelock when |clkB| > fine_exit. */
	int32_t clkB_fine_enter_ns;
	int32_t clkB_fine_exit_ns;
	/* Require |clkB| < fine_enter continuously for this many seconds before finelock (avoids qErr at clkB zero-cross). */
	float clkB_fine_enter_hold_s;
	/* Finelock: require |qErr| < this (ps) to enter; leave finelock when |qErr| >= this (avoids TIM-TP flip). */
	int32_t qerr_abs_fine_max_ps;
	/* Initial DAC / manual "int" command (PWM counts). */
	float integrator;
	/* Fine-loop strategy: 0=P (legacy), 1=edge tracker (learn local DAC->qErr slope, dither at edge). */
	int32_t fine_mode;
	/* Edge mode: number of TIM-TP samples to wait before scoring a DAC step (5 ~= 5 s at 1PPS). */
	int32_t fine_edge_eval_samples;
} gpsdo_tune_t;

extern gpsdo_tune_t gpsdo_tune;

typedef enum {
	GPSDO_MODE_SERVO = 0,
	/* Free-running: GPS servo does not move DAC; use dac to set manually. */
	GPSDO_MODE_FREE = 1,
} gpsdo_run_mode_t;

extern gpsdo_run_mode_t gpsdo_run_mode;

typedef void (*gpsdo_manual_dac_fn)(uint32_t pwm_counts);

void gpsdo_tune_init_defaults(void);
void gpsdo_tune_set_manual_dac_callback(gpsdo_manual_dac_fn fn);
/* Parse one null-terminated line (no \\r). Returns true if a tune command was recognized. */
bool gpsdo_tune_try_cli_line(const char* line);
void gpsdo_tune_dump(void);
void gpsdo_tune_print_version_banner(void);
