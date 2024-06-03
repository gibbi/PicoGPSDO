#include "gpsdo_tune.h"

#include "firmware_version.h"
#include "pico/stdio.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

gpsdo_tune_t gpsdo_tune;
gpsdo_run_mode_t gpsdo_run_mode = GPSDO_MODE_SERVO;

static gpsdo_manual_dac_fn manual_dac_cb;

void gpsdo_tune_set_manual_dac_callback(gpsdo_manual_dac_fn fn) {
	manual_dac_cb = fn;
}

void gpsdo_tune_init_defaults(void) {
	/* Fine P-only on ps-quantized qErr + integer PWM → limit cycles; defaults favor stability over raw speed. */
	gpsdo_tune.kp_qerr = 0.08f;
	gpsdo_tune.kp_coarse = 0.08f;
	gpsdo_tune.facc_max_gate = 3000;
	gpsdo_tune.min_sv = 4;
	gpsdo_tune.clkB_fine_enter_ns = 50;
	gpsdo_tune.clkB_fine_exit_ns = 300;
	gpsdo_tune.clkB_fine_enter_hold_s = 30.0f;
	gpsdo_tune.qerr_abs_fine_max_ps = 5000;
	gpsdo_tune.integrator = 17950.0f;
	gpsdo_tune.fine_mode = 0;
	gpsdo_tune.fine_edge_eval_samples = 5;
	gpsdo_run_mode = GPSDO_MODE_SERVO;
}

void gpsdo_tune_print_version_banner(void) {
	printf("[ver] %s\n", FIRMWARE_VERSION_STRING);
	stdio_flush();
}

static void copy_first_word_lower(const char* line, char* cmd, size_t cmdsz) {
	size_t i = 0;
	while (*line && isspace((unsigned char)*line)) {
		line++;
	}
	while (*line && !isspace((unsigned char)*line) && i < cmdsz - 1) {
		cmd[i++] = (char)tolower((unsigned char)*line++);
	}
	cmd[i] = '\0';
}

static const char* skip_first_word(const char* line) {
	while (*line && isspace((unsigned char)*line)) {
		line++;
	}
	while (*line && !isspace((unsigned char)*line)) {
		line++;
	}
	while (*line && isspace((unsigned char)*line)) {
		line++;
	}
	return line;
}

void gpsdo_tune_dump(void) {
	const char* mode_s = (gpsdo_run_mode == GPSDO_MODE_FREE) ? "F" : "S";
	printf("[d] %s %s\n", FIRMWARE_VERSION_STRING, mode_s);
	printf("kp=%.4g kc=%.4g int=%.5g\n", (double)gpsdo_tune.kp_qerr, (double)gpsdo_tune.kp_coarse, (double)gpsdo_tune.integrator);
	printf("facc=%ld min_sv=%ld fe=%ld fx=%ld fe_hold=%.1fs\n", (long)gpsdo_tune.facc_max_gate, (long)gpsdo_tune.min_sv,
			(long)gpsdo_tune.clkB_fine_enter_ns, (long)gpsdo_tune.clkB_fine_exit_ns, (double)gpsdo_tune.clkB_fine_enter_hold_s);
	printf("qmax=%ld fine_mode=%ld edge_eval=%ld\n", (long)gpsdo_tune.qerr_abs_fine_max_ps, (long)gpsdo_tune.fine_mode,
			(long)gpsdo_tune.fine_edge_eval_samples);
	printf("ver|mode|dac|kp|kp_coarse|facc|min_sv|fe|fx|fe_hold|fine_qmax|fine_mode|fine_edge_eval|int|dump|help\n");
	stdio_flush();
}

static void print_help(void) {
	printf(
			"[h] one line/cmd:\n"
			"ver  mode free|servo  dac <n> (free only)\n"
			"kp kp_coarse\n"
			"facc min_sv fine_enter fine_exit fine_enter_hold fine_qmax\n"
			"fine_mode p|edge  fine_edge_eval <samples>\n"
			"int  dump help\n");
	stdio_flush();
}

bool gpsdo_tune_try_cli_line(const char* line_in) {
	const char* line = line_in;
	while (*line && isspace((unsigned char)*line)) {
		line++;
	}
	if (*line == '\0') {
		return false;
	}

	char cmd[20];
	copy_first_word_lower(line, cmd, sizeof(cmd));

	if (strcmp(cmd, "help") == 0) {
		print_help();
		return true;
	}
	if (strcmp(cmd, "dump") == 0) {
		gpsdo_tune_dump();
		return true;
	}
	if (strcmp(cmd, "ver") == 0) {
		printf("%s\n", FIRMWARE_VERSION_STRING);
		stdio_flush();
		return true;
	}

	const char* arg = skip_first_word(line);
	float f;
	long lo;
	unsigned long ul;
	int n;

	if (strcmp(cmd, "mode") == 0) {
		char sub[16];
		copy_first_word_lower(arg, sub, sizeof(sub));
		if (strcmp(sub, "free") == 0) {
			gpsdo_run_mode = GPSDO_MODE_FREE;
			printf("[g] F\n");
			stdio_flush();
			return true;
		}
		if (strcmp(sub, "servo") == 0) {
			gpsdo_run_mode = GPSDO_MODE_SERVO;
			printf("[g] S\n");
			stdio_flush();
			return true;
		}
		printf("[g] mode free|servo\n");
		stdio_flush();
		return true;
	}

	if (strcmp(cmd, "dac") == 0) {
		n = sscanf(arg, "%lu", &ul);
		if (n == 1) {
			if (gpsdo_run_mode != GPSDO_MODE_FREE) {
				printf("[g] need mode free\n");
				stdio_flush();
				return true;
			}
			if (manual_dac_cb) {
				manual_dac_cb((uint32_t)ul);
			}
			printf("[g] dac %lu\n", ul);
			stdio_flush();
			return true;
		}
	}

	if (strcmp(cmd, "kp") == 0) {
		n = sscanf(arg, "%f", &f);
		if (n == 1) {
			gpsdo_tune.kp_qerr = f;
			printf("[t] kp %.4g\n", (double)gpsdo_tune.kp_qerr);
			stdio_flush();
			return true;
		}
	}
	if (strcmp(cmd, "kp_coarse") == 0) {
		n = sscanf(arg, "%f", &f);
		if (n == 1) {
			gpsdo_tune.kp_coarse = f;
			printf("[t] kc %.4g\n", (double)gpsdo_tune.kp_coarse);
			stdio_flush();
			return true;
		}
	}
	if (strcmp(cmd, "facc") == 0) {
		n = sscanf(arg, "%ld", &lo);
		if (n == 1) {
			gpsdo_tune.facc_max_gate = (int32_t)lo;
			printf("[t] facc %ld\n", (long)gpsdo_tune.facc_max_gate);
			stdio_flush();
			return true;
		}
	}
	if (strcmp(cmd, "min_sv") == 0) {
		n = sscanf(arg, "%ld", &lo);
		if (n == 1 && lo >= 0) {
			gpsdo_tune.min_sv = (int32_t)lo;
			printf("[t] min_sv %ld\n", (long)gpsdo_tune.min_sv);
			stdio_flush();
			return true;
		}
	}
	if (strcmp(cmd, "fine_enter") == 0) {
		n = sscanf(arg, "%ld", &lo);
		if (n == 1) {
			gpsdo_tune.clkB_fine_enter_ns = (int32_t)lo;
			printf("[t] fe %ld\n", (long)gpsdo_tune.clkB_fine_enter_ns);
			stdio_flush();
			return true;
		}
	}
	if (strcmp(cmd, "fine_exit") == 0) {
		n = sscanf(arg, "%ld", &lo);
		if (n == 1) {
			gpsdo_tune.clkB_fine_exit_ns = (int32_t)lo;
			printf("[t] fx %ld\n", (long)gpsdo_tune.clkB_fine_exit_ns);
			stdio_flush();
			return true;
		}
	}
	if (strcmp(cmd, "fine_enter_hold") == 0) {
		n = sscanf(arg, "%f", &f);
		if (n == 1 && f >= 0.0f) {
			gpsdo_tune.clkB_fine_enter_hold_s = f;
			printf("[t] fe_hold %.3g\n", (double)gpsdo_tune.clkB_fine_enter_hold_s);
			stdio_flush();
			return true;
		}
	}
	if (strcmp(cmd, "fine_qmax") == 0) {
		n = sscanf(arg, "%ld", &lo);
		if (n == 1 && lo >= 0) {
			gpsdo_tune.qerr_abs_fine_max_ps = (int32_t)lo;
			printf("[t] qmax %ld\n", (long)gpsdo_tune.qerr_abs_fine_max_ps);
			stdio_flush();
			return true;
		}
	}
	if (strcmp(cmd, "fine_mode") == 0) {
		char sub[16];
		copy_first_word_lower(arg, sub, sizeof(sub));
		if (strcmp(sub, "p") == 0) {
			gpsdo_tune.fine_mode = 0;
			printf("[t] fine_mode p\n");
			stdio_flush();
			return true;
		}
		if (strcmp(sub, "edge") == 0) {
			gpsdo_tune.fine_mode = 1;
			printf("[t] fine_mode edge\n");
			stdio_flush();
			return true;
		}
		printf("[t] fine_mode p|edge\n");
		stdio_flush();
		return true;
	}
	if (strcmp(cmd, "fine_edge_eval") == 0) {
		n = sscanf(arg, "%ld", &lo);
		if (n == 1 && lo >= 1) {
			gpsdo_tune.fine_edge_eval_samples = (int32_t)lo;
			printf("[t] edge_eval %ld\n", (long)gpsdo_tune.fine_edge_eval_samples);
			stdio_flush();
			return true;
		}
	}
	if (strcmp(cmd, "int") == 0) {
		n = sscanf(arg, "%f", &f);
		if (n == 1) {
			gpsdo_tune.integrator = f;
			printf("[t] int %.5g\n", (double)gpsdo_tune.integrator);
			stdio_flush();
			return true;
		}
	}

	return false;
}
