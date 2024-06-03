#pragma once

#include "gpsdo_tune.h"
#include "ubx_clean.h"

#include <stdbool.h>

/* Update derived NAV-CLOCK terms used by coarse steering. Call when newBias is raised. */
void gpsdo_control_on_new_bias(gps_data* data);
void gpsdo_control_reset_state(void);
float gpsdo_control_intended_drift(void);

/* Finelock hysteresis update (100 ms loop cadence). */
bool gpsdo_control_update_finelock(const gps_data* data, const gpsdo_tune_t* tune, bool finelock, float* finelock_enter_accum_s);

/* JSON telemetry tr bitmask helper. */
unsigned gpsdo_control_track_flags(const gps_data* data, const gpsdo_tune_t* tune);

/* Fine / coarse loop controller outputs in ns before actuator mapping. */
float gpsdo_control_fine_u_ns(const gps_data* data, const gpsdo_tune_t* tune);
float gpsdo_control_coarse_u_ns(const gps_data* data, const gpsdo_tune_t* tune);
