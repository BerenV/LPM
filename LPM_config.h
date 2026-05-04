#pragma once

#include <Arduino.h>

// Do not name this `adcResolution` — it breaks any `extern int adcResolution` declarations.
#ifndef LPM_ADC_RESOLUTION_BITS
#define LPM_ADC_RESOLUTION_BITS 10
#endif
#define Z_STEPS_PER_DMM 16
#define Y_STEPS_PER_DMM 16
#define X_STEPS_PER_DMM 2.67346886
#define TOOL_OFFSET (250 * Z_STEPS_PER_DMM)
#define Z_HOME_OFFSET 11570
#define Y_HOME_OFFSET 200
#define LASER1_OFFSET 640
#define SPINDLE_CURR 2000
#define DRILL_Z_VELOCITY 500
#define Z_PARK (-50)
#define SASH_OFFSET (-231)
#define REV_SASH_OFFSET 63

// Set 0 to stop after precision homing (commissioning). Set 1 for full drill + eject cycle.
#ifndef LPM_ENABLE_DRILL_CYCLE
#define LPM_ENABLE_DRILL_CYCLE 1
#endif

// Minimum manifold pressure (psi) before motion is paused
#ifndef LPM_MANIFOLD_PRESSURE_MIN_PSI
#define LPM_MANIFOLD_PRESSURE_MIN_PSI 25.0
#endif

// Rough X reference after beam1 break: counts added to PositionRefSet (tune on machine)
#ifndef LPM_BEAM1_ROUGH_REF_COUNTS
#define LPM_BEAM1_ROUGH_REF_COUNTS 0
#endif

// Expected spacing beam2 to beam3 (counts) — from machine notes; used as sanity check
#ifndef LPM_BEAM2_TO_BEAM3_EXPECT_COUNTS
#define LPM_BEAM2_TO_BEAM3_EXPECT_COUNTS 684
#endif

#ifndef LPM_BEAM23_SPACING_TOLERANCE_COUNTS
#define LPM_BEAM23_SPACING_TOLERANCE_COUNTS 200
#endif

// Commissioning: X velocity (counts/s) while seeking beam1
#ifndef LPM_COMMISSION_FEED_VELOCITY
#define LPM_COMMISSION_FEED_VELOCITY (1000 * 8)
#endif

// SD: use SD.begin() with no args (ClearCore board support package sets CS).

// Runtime overrides (may be loaded from SD)
extern double g_pressureMinPsi;
extern int32_t gBeam1RoughRefCounts;
