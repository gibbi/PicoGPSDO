#include "gpsdo_control.h"

#include <math.h>
#include <stdlib.h>

static float s_intended_drift;

void gpsdo_control_reset_state(void) {
	s_intended_drift = 0.0f;
}

float gpsdo_control_intended_drift(void) {
	return s_intended_drift;
}

void gpsdo_control_on_new_bias(gps_data* data) {
	int32_t new_clkB = data->clkB;
	/* UBX parser snapshots the previous clkB into lastQerr before updating clkB. */
	int32_t prev_clkB = (int32_t)data->lastQerr;
	int32_t delta = new_clkB - prev_clkB;
	int32_t duration_ms = (int32_t)(data->iTOW - data->lastChangedBiasTOW);
	float duration_s = (float)duration_ms / 1000.0f;

	if (prev_clkB != new_clkB) {
		data->lastChangedBiasTOW = data->iTOW;
		if (duration_s != 0.0f) {
			data->clkD_bias = ((float)delta - s_intended_drift * duration_s) / duration_s;
		}
		s_intended_drift = (float)new_clkB / 50.0f * -1.0f;
		if (s_intended_drift > 800.0f) {
			s_intended_drift = 800.0f;
		}
		if (s_intended_drift < -800.0f) {
			s_intended_drift = -800.0f;
		}
		data->adjustDac = true;
	}
}

bool gpsdo_control_update_finelock(const gps_data* data, const gpsdo_tune_t* tune, bool finelock, float* finelock_enter_accum_s) {
	const int32_t ab = abs(data->clkB);
	const int32_t aq = data->qErr >= 0 ? data->qErr : -data->qErr;
	const int32_t qmax_ps = tune->qerr_abs_fine_max_ps;
	const bool q_ok = (qmax_ps <= 0) || (aq < qmax_ps);

	if (ab > tune->clkB_fine_exit_ns) {
		finelock = false;
		*finelock_enter_accum_s = 0.0f;
	} else if (finelock && qmax_ps > 0 && aq >= qmax_ps) {
		finelock = false;
		*finelock_enter_accum_s = 0.0f;
	} else if (!finelock) {
		if (ab < tune->clkB_fine_enter_ns && q_ok) {
			*finelock_enter_accum_s += 0.1f;
			if (*finelock_enter_accum_s >= tune->clkB_fine_enter_hold_s) {
				finelock = true;
			}
		} else {
			*finelock_enter_accum_s = 0.0f;
		}
	}
	return finelock;
}

unsigned gpsdo_control_track_flags(const gps_data* data, const gpsdo_tune_t* tune) {
	unsigned tr = 0;
	if (data->fix == 0) {
		tr |= 1u;
	}
	if (tune->min_sv > 0 && (unsigned)data->satellites_visible < (unsigned)tune->min_sv) {
		tr |= 2u;
	}
	if (tune->facc_max_gate > 0 && data->fAcc > (uint32_t)tune->facc_max_gate) {
		tr |= 4u;
	}
	return tr;
}

float gpsdo_control_fine_u_ns(const gps_data* data, const gpsdo_tune_t* tune) {
	float e_ns = (float)data->qErr * 1e-3f;
	return tune->kp_qerr * e_ns;
}

float gpsdo_control_coarse_u_ns(const gps_data* data, const gpsdo_tune_t* tune) {
	return data->clkD_bias * tune->kp_coarse;
}
