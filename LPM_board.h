#pragma once

#include "ClearCore.h"
#include "LPM_axes.h"
#include <Arduino.h>

#define spindleSpeedPin CLEARCORE_PIN_IO0
#define power4hubPin CLEARCORE_PIN_IO1
#define EstopPin CLEARCORE_PIN_A9
#define spindle1RPMPin CLEARCORE_PIN_A10
#define manifoldPressurePin CLEARCORE_PIN_A12

#define spindleAlarmPin CLEARCORE_PIN_CCIOA1
#define spindleEnPin CLEARCORE_PIN_CCIOA2
#define spindleDirPin CLEARCORE_PIN_CCIOA3
#define greenMastPin CLEARCORE_PIN_CCIOA4
#define yellowMastPin CLEARCORE_PIN_CCIOA5
#define redMastPin CLEARCORE_PIN_CCIOA6
#define buzzMastPin CLEARCORE_PIN_CCIOA7

#define laser1Pin CLEARCORE_PIN_CCIOC0
#define laser2Pin CLEARCORE_PIN_CCIOC1
#define laser3Pin CLEARCORE_PIN_CCIOC2
#define laser4Pin CLEARCORE_PIN_CCIOC3
#define beam4Pin CLEARCORE_PIN_CCIOC4
#define beam3Pin CLEARCORE_PIN_CCIOC5
#define beam2Pin CLEARCORE_PIN_CCIOC6
#define beam1Pin CLEARCORE_PIN_CCIOC7

extern uint8_t ccioBoardCount;
extern uint8_t ccioPinCount;
extern int32_t clampPins[];
constexpr size_t CLAMP_COUNT = 8;

void setupPins();

inline void clamp(int index, bool state) {
	if (index < 1 || index > (int)CLAMP_COUNT) {
		return;
	}
	digitalWrite(clampPins[index - 1], state);
}

inline void laser1(bool s) { digitalWrite(laser1Pin, s); }
inline void laser2(bool s) { digitalWrite(laser2Pin, s); }
inline void laser3(bool s) { digitalWrite(laser3Pin, s); }
inline void laser4(bool s) { digitalWrite(laser4Pin, s); }

inline bool beam1() { return digitalRead(beam1Pin); }
inline bool beam2() { return digitalRead(beam2Pin); }
inline bool beam3() { return digitalRead(beam3Pin); }
inline bool beam4() { return digitalRead(beam4Pin); }
