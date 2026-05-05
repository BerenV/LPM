#pragma once

#include <Arduino.h>

// Do not name this `adcResolution` — it breaks any `extern int adcResolution` declarations.
#ifndef LPM_ADC_RESOLUTION_BITS
#define LPM_ADC_RESOLUTION_BITS 10
#endif
#define Z_STEPS_PER_DMM 16
#define Y_STEPS_PER_DMM 16
#define X_STEPS_PER_DMM 2.67346886

// --- Motor-count calibration (use as-is with *StartCounts / distance moves; do NOT × STEPS_PER_DMM) ---
#define Y_HOME_OFFSET_COUNTS 200    // Y: encoder counts from machine home to roller edge
#define Z_HOME_OFFSET_COUNTS 11570   // Z: counts from home to collet aligned with lower rollers
#define TOOL_OFFSET_COUNTS (250 * Z_STEPS_PER_DMM) // tip-to-collet in counts (250 DMM × spr)

// Legacy aliases (same values — older helpers confused “DMM” parameter names with count offsets)
#define Y_HOME_OFFSET Y_HOME_OFFSET_COUNTS
#define Z_HOME_OFFSET Z_HOME_OFFSET_COUNTS
#define TOOL_OFFSET TOOL_OFFSET_COUNTS

#define LASER1_OFFSET 640 // counts (beam / mechanical calibration)
#define SPINDLE_CURR 2000
#define DRILL_Z_VELOCITY 500
// Program coordinates in DMM (0.1 mm); multiply by *_STEPS_PER_DMM for counts when needed
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
// +1 or -1 — flip if the stick feeds the wrong way for your wiring/polarity
#ifndef LPM_COMMISSION_FEED_SIGN
#define LPM_COMMISSION_FEED_SIGN 1
#endif

// Abort X enable in setup if HLFB never asserts (disconnected motor, bad HLFB). 0 = no timeout.
#ifndef LPM_X_ENABLE_TIMEOUT_MS
#define LPM_X_ENABLE_TIMEOUT_MS 30000
#endif

// While 1, low manifold pressure does NOT pause motion until precision homing — avoids blocking
// beam1 feed when pneumatics are off or sensor reads low during bench bring-up. Set 0 for production.
#ifndef LPM_ALLOW_LOW_AIR_DURING_BEAM1_FEED
#define LPM_ALLOW_LOW_AIR_DURING_BEAM1_FEED 1
#endif

// SD: use SD.begin() with no args (ClearCore board support package sets CS).

// Runtime overrides (may be loaded from SD)
extern double g_pressureMinPsi;
extern int32_t gBeam1RoughRefCounts;
