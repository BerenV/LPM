#include "LPM_board.h"
#include "LPM_config.h"

#include <cstdio>

#define MAX_MSG_LEN 80
extern char msg[MAX_MSG_LEN + 1];

uint8_t ccioBoardCount;
uint8_t ccioPinCount;

int32_t clampPins[] = {
	CLEARCORE_PIN_CCIOB0, CLEARCORE_PIN_CCIOB1, CLEARCORE_PIN_CCIOB2, CLEARCORE_PIN_CCIOB3,
	CLEARCORE_PIN_CCIOB4, CLEARCORE_PIN_CCIOB5, CLEARCORE_PIN_CCIOB6, CLEARCORE_PIN_CCIOB7
};

static_assert(sizeof(clampPins) / sizeof(clampPins[0]) == CLAMP_COUNT, "clampPins size");

void setupPins() {
	analogReadResolution(LPM_ADC_RESOLUTION_BITS);
	CcioPort.Mode(Connector::CCIO);
	CcioPort.PortOpen();
	ccioBoardCount = CcioMgr.CcioCount();
	ccioPinCount = ccioBoardCount * CCIO_PINS_PER_BOARD;
	snprintf(msg, MAX_MSG_LEN, "Discovered %d CCIO-8 board", ccioBoardCount);
	Serial.print(msg);
	if (ccioBoardCount != 1) {
		Serial.print("s");
	}
	Serial.println("...");
	Serial.println();

	for (size_t i = 0; i < CLAMP_COUNT; i++) {
		pinMode(clampPins[i], OUTPUT);
	}
	pinMode(laser1Pin, OUTPUT);
	pinMode(laser2Pin, OUTPUT);
	pinMode(laser3Pin, OUTPUT);
	pinMode(laser4Pin, OUTPUT);
	pinMode(beam1Pin, INPUT);
	pinMode(beam2Pin, INPUT);
	pinMode(beam3Pin, INPUT);
	pinMode(beam4Pin, INPUT);
	pinMode(power4hubPin, OUTPUT);
	pinMode(buzzMastPin, OUTPUT);
	pinMode(redMastPin, OUTPUT);
	pinMode(yellowMastPin, OUTPUT);
	pinMode(greenMastPin, OUTPUT);
	pinMode(spindleSpeedPin, OUTPUT);
	pinMode(spindleDirPin, OUTPUT);
	pinMode(spindleEnPin, OUTPUT);
	pinMode(spindleAlarmPin, INPUT);
	pinMode(EstopPin, INPUT);

	Xaxis.EStopConnector(EstopPin);
	Yaxis.EStopConnector(EstopPin);
	Zaxis.EStopConnector(EstopPin);

	bool laserError = false;
	if (!digitalRead(beam1Pin)) {
		laserError = true;
	}
	digitalWrite(laser1Pin, HIGH);
	delay(100);
	if (digitalRead(beam1Pin)) {
		laserError = true;
	}
	digitalWrite(laser1Pin, LOW);

	if (!digitalRead(beam2Pin)) {
		laserError = true;
	}
	digitalWrite(laser2Pin, HIGH);
	delay(100);
	if (digitalRead(beam2Pin)) {
		laserError = true;
	}
	digitalWrite(laser2Pin, LOW);

	if (!digitalRead(beam3Pin)) {
		laserError = true;
	}
	digitalWrite(laser3Pin, HIGH);
	delay(100);
	if (digitalRead(beam3Pin)) {
		laserError = true;
	}
	digitalWrite(laser3Pin, LOW);

	if (laserError) {
		Serial.println("Error during laser test");
	} else {
		Serial.println("Laser test passed");
	}
	digitalWrite(laser1Pin, HIGH);
	digitalWrite(laser2Pin, HIGH);
	digitalWrite(laser3Pin, HIGH);
}
