#include "ClearCore.h"
#define Xaxis ConnectorM0
#define Yaxis ConnectorM1
#define Zaxis ConnectorM2
#define CcioPort ConnectorCOM0
#define baudRate 115200
// Supported adcResolution values are 8, 10, and 12
#define adcResolution 10

// decided to use 0.1mm (decimillimeter) as the standard unit to avoid floats where possible
#define Z_STEPS_PER_DMM 16
#define Y_STEPS_PER_DMM 16
#define X_STEPS_PER_DMM 2.67346886         // 6400/(3*254*pi)	 fine tune this value for accuracy
#define TOOL_OFFSET 250 * Z_STEPS_PER_DMM  // tip of tool to collet
#define Z_HOME_OFFSET 11410                // in counts, from home to collet lined up with lower rollers
#define Y_HOME_OFFSET 200                  // in counts, from home to edge of drive rollers
#define LASER1_OFFSET 640                  // in counts
// #define SPINDLE_CURR 2000 // 20mA, full speed
#define SPINDLE_CURR 900
#define DRILL_Z_VELOCITY 400  // counts per second
#define Z_PARK -50            // DMM

#define Yoffset1 240  // DMM from Y home to middle of hardware track
#define Zstart1 310
#define Zstop1 400
#define Yoffset2 480  // other holes for pan head screws
#define Zstart2 230
#define Zstop2 280
#define hole1 565  // DMM from tip
#define hole2 1065
#define hole3 1065
#define hole4 465
#define Z_INTERMEDIATE_PARK 180  // to not collide when moving Y?


//~~~~~~~~~ pin definitions ~~~~~~~~~
// ClearCore local pins:
#define spindleSpeedPin CLEARCORE_PIN_IO0  // 0-20 mA out is translated to 0-10V by shunt resistor at spindle drive
#define power4hubPin CLEARCORE_PIN_IO1     // switches aux power to hub through relay, to allow forcing axes to home on startup
// #define beam1Pin CLEARCORE_PIN_IO2 // TEMPORARY
#define EstopPin CLEARCORE_PIN_A9              // connected to actual, normally closed, active low circuit
#define spindle1RPMPin CLEARCORE_PIN_A10       // provides feedback on actual speed, RPM=2*P/N*60 where N=# poles, P=input freq (Hz)
#define manifoldPressurePin CLEARCORE_PIN_A12  // 4-20 mA corresponds to 0-100 psi
uint8_t ccioBoardCount;                        // Store the number of connected CCIO-8 boards here.
uint8_t ccioPinCount;                          // Store the number of connected CCIO-8 pins here.
// first CCIO (control cabinet):
// #define eStoppedPin CLEARCORE_PIN_CCIOA0 // future momentary button (lighted?) on eStop box
#define spindleAlarmPin CLEARCORE_PIN_CCIOA1
#define spindleEnPin CLEARCORE_PIN_CCIOA2
#define spindleDirPin CLEARCORE_PIN_CCIOA3
#define greenMastPin CLEARCORE_PIN_CCIOA4
#define yellowMastPin CLEARCORE_PIN_CCIOA5
#define redMastPin CLEARCORE_PIN_CCIOA6
#define buzzMastPin CLEARCORE_PIN_CCIOA7
// second CCIO (pneumatics):
int32_t clampPins[] = { CLEARCORE_PIN_CCIOB0, CLEARCORE_PIN_CCIOB1, CLEARCORE_PIN_CCIOB2, CLEARCORE_PIN_CCIOB3, CLEARCORE_PIN_CCIOB4, CLEARCORE_PIN_CCIOB5, CLEARCORE_PIN_CCIOB6, CLEARCORE_PIN_CCIOB7 };
// #define clamp1Pin CLEARCORE_PIN_CCIOB0
// #define clamp2Pin CLEARCORE_PIN_CCIOB1
// #define clamp3Pin CLEARCORE_PIN_CCIOB2
// #define clamp4Pin CLEARCORE_PIN_CCIOB3
// #define clamp5Pin CLEARCORE_PIN_CCIOB4
// #define clamp6Pin CLEARCORE_PIN_CCIOB5
// #define clamp7Pin CLEARCORE_PIN_CCIOB6
// #define clamp8Pin CLEARCORE_PIN_CCIOB7
// third CCIO (machine sensors):
#define laser1Pin CLEARCORE_PIN_CCIOC0  // laser emitters
#define laser2Pin CLEARCORE_PIN_CCIOC1
#define laser3Pin CLEARCORE_PIN_CCIOC2
#define laser4Pin CLEARCORE_PIN_CCIOC3
#define beam4Pin CLEARCORE_PIN_CCIOC4  // laser detectors
#define beam3Pin CLEARCORE_PIN_CCIOC5
#define beam2Pin CLEARCORE_PIN_CCIOC6
#define beam1Pin CLEARCORE_PIN_CCIOC7
// function wrappers for readability
constexpr size_t CLAMP_COUNT = sizeof(clampPins) / sizeof(clampPins[0]);
inline void clamp(int index, bool state) {
	if (index < 1 || index >= CLAMP_COUNT + 1) return;  // is that right?
	digitalWrite(clampPins[index - 1], state);
}
inline void unclamp() {
	for (int i = 1; i < 5; i++) { clamp(i, 0); }
}
void laser1(bool s) {
	digitalWrite(laser1Pin, s);
}
void laser2(bool s) {
	digitalWrite(laser2Pin, s);
}
void laser3(bool s) {
	digitalWrite(laser3Pin, s);
}
void laser4(bool s) {
	digitalWrite(laser4Pin, s);
}
bool beam1() {
	return digitalRead(beam1Pin);
}  // true when beam is broken (LED on)
bool beam2() {
	return digitalRead(beam2Pin);
}
bool beam3() {
	return digitalRead(beam3Pin);
}
bool beam4() {
	return digitalRead(beam4Pin);
}

enum MotionState {
	IDLE,
	MOVING,
	PAUSED_ESTOP,
	PAUSED_PRESSURE,
	PAUSED_MOTOR_FAULT
};
MotionState motionState = IDLE;

// SASH OFFSETS (everything in counts)
#define beam1Offset 6837
#define beam1OffsetRev 9111
// #define SASH_OFFSET 2513 // counts, laser broken to tip of sash
// increasing absolute value of these will move holes closer to end of stick
#define SASH_OFFSET 1890      // counts, tip of sash to beam 3
#define REV_SASH_OFFSET -470  // counts, tip of sash to beam 2
// laser 1 is first one, laser 2 is upper beam and laser 3 is lower beam
// entering wheels to laser 1: 2848
// laser 1 to tip at spindle center: 6837
// tip at spindle center to laser 3: 1872
// laser 3 to laser 2: 641
// clamp 2 can come down as soon as tip is past spindle
// clamp 3 down as soon as tip is 3315 counts past spindle (right before third wheels grip)
// other end of stick at laser 1 to spindle center: 9111
// other end of stick at laser 1 to clamp 2 raising: 5623
// other end of stick at spindle center to beam 2: -450
// other end of stick at laser 1 to laser 2: 8704
// clamp 3 needs to raise as soon as tip is past spindle
// beam 2 to beam 3: 684
// beam 3 to stick leaving wheels: 9202



// This example has built-in functionality to automatically clear motor alerts,
//	including motor shutdowns. Any uncleared alert will cancel and disallow motion.
// WARNING: enabling automatic alert handling will clear alerts immediately when
//	encountered and return a motor to a state in which motion is allowed. Before
//	enabling this functionality, be sure to understand this behavior and ensure
//	your system will not enter an unsafe state.
// To enable automatic alert handling, #define HANDLE_ALERTS (1)
// To disable automatic alert handling, #define HANDLE_ALERTS (0)
#define HANDLE_ALERTS (0)

// These will be used to format the text that is printed to the serial port.
#define MAX_MSG_LEN 80
char msg[MAX_MSG_LEN + 1];

// Define the initial velocity and acceleration limits
int xVelLim = 3000 * 8;     // pulses per sec
int xAccelLim = 10000 * 8;  // pulses per sec^2
int yVelLim = 15000;        // pulses per sec
int yAccelLim = 1000000;    // pulses per sec^2
int zVelLim = 15000;        // pulses per sec
int zAccelLim = 1000000;    // pulses per sec^2
bool dir = 0;               // for spindle test

// Declares user-defined helper functions (prototypes).
// The definition/implementations of these functions are at the bottom of the sketch.
void clampManager();
void drillSashEnd1();
void drillSashEndRev();
void drillPeck();
void startSpindle();
void stopSpindle();
void homeStick();
void revHomeStick();
void spindleTest();
bool XMoveAtVelocity(int32_t velocity);
bool XMoveDistance(int distance);
bool XMoveAbsolutePosition(int position);
// bool YMoveAtVelocity(int32_t velocity);
bool YMoveDistance(int distance);
bool YMoveAbsolutePosition(int position);
// bool ZMoveAtVelocity(int32_t velocity);
bool ZMoveDistance(int distance);
bool ZMoveAbsolutePosition(int position);
void PrintAlerts();
void HandleAlertsX();
void HandleAlertsY();
void HandleAlertsZ();
void setupMotors();
void enableX();
void homeY();
void homeZ();
double readAirPressure();
void setupPins();
// TODO: find out if there's actually any advantage to declaring these at the top of the sketch

void setup() {
	// Sets up serial communication and waits up to "timeout" seconds for a port to open.
	Serial.begin(baudRate);
	// uint32_t timeout = 1000;
	// uint32_t startTime = millis();
	// while (!Serial && millis() - startTime < timeout) {
	// 	continue;
	// }
	// Serial.println();

	digitalWrite(power4hubPin, LOW);  // do this to force motors to reset and home
	setupPins();
	// delay(100); // necessary to wait a sec for CCIO digitalRead to work
	delay(2000);
	digitalWrite(power4hubPin, HIGH);

	digitalWrite(redMastPin, !digitalRead(EstopPin));
	if (!digitalRead(EstopPin)) {
		Serial.println("Waiting for E-stop disengagement");
		while (!digitalRead(EstopPin)) {
			// Serial.print(".");
			delay(100);
		}
		digitalWrite(redMastPin, !digitalRead(EstopPin));
	}
	digitalWrite(yellowMastPin, HIGH);
	delay(2000);  // allow time for IPC to power up
	setupMotors();
	digitalWrite(yellowMastPin, LOW);
	digitalWrite(greenMastPin, HIGH);  // turns on green "power" light

	// ZMoveAbsolutePosition(0);
	// while(true) { // don't go on to loop for now
	// 	digitalWrite(redMastPin, !digitalRead(EstopPin));
	// 	// Serial.println(digitalRead(EstopPin));
	// 	readAirPressure();
	// 	Serial.print(beam1());
	// 	Serial.print(beam2());
	// 	Serial.println(beam3());
	// 	delay(1000);
	// }

	// Begin a 100ms on/200ms off pulse on the first CCIO-8 board's
	// connector 0 output that will complete 20 cycles and prevent further
	// code execution until the cycles are complete.
	// CcioMgr.PinByIndex(CLEARCORE_PIN_CCIOA5)->OutputPulsesStart(100, 200, 20, true);
	Serial.println("Loop start");
}

void loop() {
	digitalWrite(redMastPin, !digitalRead(EstopPin));
	homeStick();
	CcioMgr.PinByIndex(yellowMastPin)->OutputPulsesStart(800, 800, 0, false);  // yellow flashing
	drillSashEnd1();
	revHomeStick();
	drillSashEndRev();
	unclamp();
	if (beam2()) {
		while (beam2()) {
			Xaxis.MoveVelocity(4000 * 8);
		}
	}
	Xaxis.VelMax(2000 * 8);                                                  // safe speed? don't want to launch stick
	XMoveDistance(1370 * 8);                                                 // should eject stick
	Xaxis.VelMax(xVelLim);                                                   // sets back to normal
	CcioMgr.PinByIndex(yellowMastPin)->OutputPulsesStop();                   // cancel yellow flashing
	CcioMgr.PinByIndex(buzzMastPin)->OutputPulsesStart(250, 250, 1, false);  // quick beep
	delay(1000);
}
// run this anytime X-axis is moving to coordinate clamping
void clampManager() {
}

void drillSashEnd1() {
	// int Yoffset1 = 240; // DMM from Y home to middle of hardware track
	// int Zstart1 = 200;
	// int Zstop1 = 400;
	// int Yoffset2 = 480; // other holes for pan head screws
	// int Zstart2 = 230;
	// int Zstop2 = 300;
	// int hole1 = 570; // DMM from tip
	// int hole2 = 1070;
	// int hole3 = 1070;
	// int hole4 = 470;
	startSpindle();
	ZMoveAbsolutePosition(Z_PARK);  // park Z to avoid collision when moving Y
	YMoveAbsolutePosition(Yoffset1);
	XMoveAbsolutePosition(hole1);
	drillPeck(Zstart1, Zstop1);
	XMoveAbsolutePosition(hole2);
	drillPeck(Zstart1, Zstop1);
	// drill other two holes
	ZMoveAbsolutePosition(Z_INTERMEDIATE_PARK);
	YMoveAbsolutePosition(Yoffset2);
	XMoveAbsolutePosition(hole3);
	drillPeck(Zstart2, Zstop2);
	XMoveAbsolutePosition(hole4);
	drillPeck(Zstart2, Zstop2);
	ZMoveAbsolutePosition(Z_INTERMEDIATE_PARK);
	stopSpindle();
}
void drillSashEndRev() {
	// int Yoffset1 = 240; // DMM from Y home to middle of hardware track
	// int Zstart1 = 300;
	// int Zstop1 = 400;
	// int Yoffset2 = 480; // other holes for pan head screws
	// int Zstart2 = 230;
	// int Zstop2 = 300;
	// int hole1 = -570; // DMM from tip
	// int hole2 = -1070;
	// int hole3 = -1070;
	// int hole4 = -470;
	startSpindle();
	ZMoveAbsolutePosition(Z_PARK);  // park Z to avoid collision when moving Y
	YMoveAbsolutePosition(Yoffset1);
	XMoveAbsolutePosition(-1 * hole1);
	drillPeck(Zstart1, Zstop1);
	XMoveAbsolutePosition(-1 * hole2);
	drillPeck(Zstart1, Zstop1);
	// drill other two holes
	ZMoveAbsolutePosition(Z_INTERMEDIATE_PARK);
	YMoveAbsolutePosition(Yoffset2);
	XMoveAbsolutePosition(-1 * hole3);
	drillPeck(Zstart2, Zstop2);
	XMoveAbsolutePosition(-1 * hole4);
	drillPeck(Zstart2, Zstop2);
	ZMoveAbsolutePosition(Z_INTERMEDIATE_PARK);
	stopSpindle();
}

void drillPeck(int Zstart, int Zstop) {  // assumes spindle is already turning
	// could add logic to check whether it's running or not, or add that to startSpindle actually
	ZMoveAbsolutePosition(Zstart);
	Zaxis.VelMax(DRILL_Z_VELOCITY);
	ZMoveAbsolutePosition(Zstop);
	Zaxis.VelMax(zVelLim);          // sets back to normal
	ZMoveAbsolutePosition(Zstart);  // quickly move back
	                                // ZMoveAbsolutePosition(Z_PARK);
}

void startSpindle() {
	digitalWrite(spindleEnPin, LOW);
	analogWrite(spindleSpeedPin, 0, CURRENT);
	digitalWrite(spindleDirPin, LOW);  // CW rotation
	analogWrite(spindleSpeedPin, SPINDLE_CURR, CURRENT);
	delay(100);
	digitalWrite(spindleEnPin, HIGH);  // engage spindle drive
	delay(1000);
}

void stopSpindle() {
	digitalWrite(spindleEnPin, LOW);
	delay(100);
}

void homeStick() {
	unclamp();                            // make sure all clamps are raised
	if (beam1() || beam2() || beam3()) {  // stick is present, reverse
		while (beam1() || beam2() || beam3()) { Xaxis.MoveVelocity(-1000 * 8); }
	}
	delay(200);  // lets stick retreat
	Xaxis.MoveStopAbrupt();
	while (!beam1()) { Xaxis.MoveVelocity(1000 * 8); }
	int firstBeam = Xaxis.PositionRefCommanded();
	while ((Xaxis.PositionRefCommanded() - firstBeam) < beam1Offset) {}
	clamp(2, 1);
	Xaxis.MoveVelocity(1000);  // slow down
	// now do more precise homing with the third beam
	while (!beam3()) {}
	Xaxis.MoveStopAbrupt();
	XMoveDistance(-30);
	delay(500);
	while (!beam3()) { Xaxis.MoveVelocity(10); }
	Xaxis.MoveStopAbrupt();
	// delay(1000); // this does nothing since the potition counter won't be changing
	Xaxis.PositionRefSet(SASH_OFFSET);  // update internal position counter
	// XMoveAbsolutePosition(0);
	// delay(5000);

	// if (digitalRead(beam2Pin)) {
	// 	while (digitalRead(beam2Pin)) { // TODO: make use of InputRisen() or fallen
	// 		Xaxis.MoveVelocity(-1000*8);
	// 	}
	// }
	// delay(50); // lets stick retreat
	// while (!digitalRead(beam2Pin)) {
	// 	Xaxis.MoveVelocity(1000*8);
	// }
	// Xaxis.MoveStopAbrupt();
	// // digitalWrite(clamp2Pin, HIGH);
	// delay(200);
	// Xaxis.AccelMax(1000*8);  // temporarily bump down for homing... why??
	// XMoveDistance(-50);
	// delay(200);
	// while (!digitalRead(beam2Pin)) {
	// 	Xaxis.MoveVelocity(20);
	// }
	// Xaxis.MoveStopAbrupt();
	// delay(1000);
	// // read current StepGenerator postition (internal to ClearCore)
	// // Xaxis.PositionRefCommanded();
	// // XMoveDistance(LASER1_OFFSET);
	// // XMoveDistance(SASH_OFFSET); // in counts
	// // Xaxis.PositionRefSet(0);    // zero internal position counter
	// Xaxis.PositionRefSet(-1*SASH_OFFSET);    // update internal position counter
	// Xaxis.AccelMax(xAccelLim);  // return this to original value
}

void revHomeStick() {
	XMoveAbsolutePosition(2000);  // 200mm, should be past laser
	clamp(3, 1);
	delay(200);
	clamp(2, 0);
	if (beam1()) { // fast traverse
		Xaxis.MoveVelocity(2000 * 8);
		while (beam1()) { delay(1); }
		int firstBeam = Xaxis.PositionRefCommanded(); // where end of stick just passed beam1
		while (abs(Xaxis.PositionRefCommanded() - firstBeam) < (beam1OffsetRev - 2000)) { delay(1); }
		Xaxis.MoveVelocity(2000);  // slow down
	}
	Serial.print("beam2 (1): "); // expected value is in parentheses
	Serial.println(beam2());
	// now do more precise homing with beam 2
	if (beam2()) { // what if not?
		Serial.println("beam 2 broken, so moving forward till it's not");
		Xaxis.MoveVelocity(2000);  // slow down
		while (beam2()) { delay(1); }
	}
	else { // retreat until beam2 is broken
	Serial.println("retreating");
		Xaxis.MoveVelocity(-2000);
		while (!beam2()) { delay(1); }
		delay(500);
		Xaxis.MoveVelocity(2000);
		while (beam2()) { delay(1); }
	}
	// Xaxis.MoveVelocity(0);
	Serial.print("beam2 (0): ");
	Serial.println(beam2());
	Xaxis.MoveStopAbrupt(); // stop when beam is not broken
	Serial.println("stopped");
	delay(500);
	Xaxis.MoveVelocity(-100);
	while (!beam2()) { delay(1); }
	delay(50);
	Xaxis.MoveStopAbrupt();
	// XMoveDistance(-150); // no idea why we have to go back so much farther on revhome
	Serial.print("beam2 (1): ");
	Serial.println(beam2());
	Serial.println("moved back until beam was broken??");
	delay(500);
	Serial.println("creeping forward");
	Xaxis.MoveVelocity(20);
	Serial.print("beam2 (1): ");
	Serial.println(beam2());
	while (beam2()) { delay(1); }
	Xaxis.MoveStopAbrupt();
	Serial.print("beam2 (0): ");
	Serial.println(beam2());
	Serial.println("setting rev_offset");
	// delay(1000); // this does nothing
	Xaxis.PositionRefSet(REV_SASH_OFFSET);  // update internal position counter
	// XMoveAbsolutePosition(0);
	// delay(5000);

	// while (digitalRead(beam2Pin)) {
	// 	Xaxis.MoveVelocity(1000*8);
	// }
	// Xaxis.MoveStopAbrupt();
	// delay(200);
	// Xaxis.AccelMax(1000*8);  // temporarily bump down for homing
	// XMoveDistance(-50);
	// delay(200);
	// while (digitalRead(beam2Pin)) {
	// 	Xaxis.MoveVelocity(20);
	// }
	// Xaxis.MoveStopAbrupt();
	// delay(1000);
	// // read current StepGenerator postition (internal to ClearCore)
	// // Xaxis.PositionRefCommanded();
	// // XMoveDistance(LASER1_OFFSET);
	// Xaxis.PositionRefSet(-1*REV_SASH_OFFSET);    // set internal position counter
	// Xaxis.AccelMax(xAccelLim);  // return this to original value
}

void spindleTest() {
	analogWrite(spindleSpeedPin, 0, CURRENT);
	digitalWrite(spindleDirPin, dir);  // CW rotation
	digitalWrite(spindleEnPin, HIGH);  // engage spindle drive
	delay(1000);
	for (int i = 0; i < 2047; i++) {
		analogWrite(spindleSpeedPin, i, CURRENT);
		delay(3);
	}
	for (int i = 2047; i > 0; i--) {
		analogWrite(spindleSpeedPin, i, CURRENT);
		delay(3);
	}
	digitalWrite(spindleEnPin, LOW);  // disengage spindle drive
	// digitalWrite(spindleDirPin, HIGH); // CCW rotation
	dir = !dir;  // flip state
	             // digitalWrite(spindleEnPin, HIGH); // engage spindle drive
	             // delay(2000);
	             // for(int i=0; i<2047; i++) {
	             // 	analogWrite(spindleSpeedPin, i, CURRENT);
	             // 	delay(3);
	             // }
	             // delay(2000);
	             // digitalWrite(spindleEnPin, LOW); // disengage spindle drive
	             // delay(2000);

	// while ((millis() % 10000) < 10) {

	// }
}

// TODO change to DMM/s
/*------------------------------------------------------------------------------
 * XMoveAtVelocity
 *
 *    Command the motor to move at the specified "velocity", in pulses/second.
 *    Prints the move status to the USB serial port
 *
 * Parameters:
 *    int velocity  - The velocity, in step pulses/sec, to command
 *
 * Returns: True/False depending on whether the move was successfully triggered.
 */
bool XMoveAtVelocity(int velocity) {
	// int velocity = round(velocityDMM*X_STEPS_PER_DMM); // round to nearest count/s cause pi
	// Check if a motor alert is currently preventing motion
	// Clear alert if configured to do so
	if (Xaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("X Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsX();
		} else {
			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("X Move canceled.");
		Serial.println();
		return false;
	}

	Serial.print("X Moving at velocity: ");
	Serial.println(velocity);

	// Command the velocity move
	Xaxis.MoveVelocity(velocity);

	// Waits for the step command to ramp up/down to the commanded velocity.
	// This time will depend on your Acceleration Limit.
	Serial.println("Ramping to speed...");
	while (!Xaxis.StatusReg().bit.AtTargetVelocity) {
		continue;
	}

	Serial.println("At Speed");
	return true;
}

// move functions still take units in counts to not break home offsets...
/*------------------------------------------------------------------------------
 * XMoveDistance
 *
 *    Command "distance" number of step pulses away from the current position
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motor has reached the commanded
 *    position)
 *
 * Parameters:
 *    int distance  - The distance, in step pulses, to move
 *
 * Returns: True/False depending on whether the move was successfully triggered.
 */
bool XMoveDistance(int distance) {  // not for use for precise positioning due to rounding
	// int distance = round(distanceDMM*X_STEPS_PER_DMM); // round to nearest count cause pi
	// Check if a motor alert is currently preventing motion
	// Clear alert if configured to do so
	if (Xaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("X Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsX();
		} else {
			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("X Move canceled.");
		Serial.println();
		return false;
	}

	Serial.print("X Moving distance: ");
	Serial.println(distance);

	// Command the move of incremental distance
	Xaxis.Move(distance);

	// Waits for HLFB to assert (signaling the move has successfully completed)
	Serial.println("Moving.. Waiting for HLFB");
	while ((!Xaxis.StepsComplete() || Xaxis.HlfbState() != MotorDriver::HLFB_ASSERTED) && !Xaxis.StatusReg().bit.AlertsPresent) {
		continue;
	}
	// Check if motor alert occurred during move
	// Clear alert if configured to do so
	if (Xaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsX();
		} else {
			Serial.println("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Motion may not have completed as expected. Proceed with caution.");
		Serial.println();
		return false;
	} else {
		Serial.println("Move Done");
		return true;
	}
}
bool YMoveDistance(int distance) {
	// int distance = distanceDMM*Y_STEPS_PER_DMM;
	// Check if a motor alert is currently preventing motion
	// Clear alert if configured to do so
	if (Yaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("Y Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsY();
		} else {
			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Y Move canceled.");
		Serial.println();
		return false;
	}

	Serial.print("Y Moving distance: ");
	Serial.println(distance);

	// Command the move of incremental distance
	Yaxis.Move(distance);

	// Waits for HLFB to assert (signaling the move has successfully completed)
	Serial.println("Moving.. Waiting for HLFB");
	while ((!Yaxis.StepsComplete() || Yaxis.HlfbState() != MotorDriver::HLFB_ASSERTED) && !Xaxis.StatusReg().bit.AlertsPresent) {
		continue;
	}
	// Check if motor alert occurred during move
	// Clear alert if configured to do so
	if (Yaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsY();
		} else {
			Serial.println("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Motion may not have completed as expected. Proceed with caution.");
		Serial.println();
		return false;
	} else {
		Serial.println("Move Done");
		return true;
	}
}
bool ZMoveDistance(int distance) {
	// int distance = distanceDMM*Z_STEPS_PER_DMM;
	// Check if a motor alert is currently preventing motion
	// Clear alert if configured to do so
	if (Zaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("Z Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsZ();
		} else {
			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Z Move canceled.");
		Serial.println();
		return false;
	}

	Serial.print("Z Moving distance: ");
	Serial.println(distance);

	// Command the move of incremental distance
	Zaxis.Move(distance);

	// Waits for HLFB to assert (signaling the move has successfully completed)
	Serial.println("Moving.. Waiting for HLFB");
	while ((!Zaxis.StepsComplete() || Zaxis.HlfbState() != MotorDriver::HLFB_ASSERTED) && !Xaxis.StatusReg().bit.AlertsPresent) {
		continue;
	}
	// Check if motor alert occurred during move
	// Clear alert if configured to do so
	if (Zaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsZ();
		} else {
			Serial.println("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Motion may not have completed as expected. Proceed with caution.");
		Serial.println();
		return false;
	} else {
		Serial.println("Move Done");
		return true;
	}
}
//------------------------------------------------------------------------------


// absolute move functions take DMM units
/*------------------------------------------------------------------------------
 * XMoveAbsolutePosition
 *
 *    Command step pulses to move the motor's current position to the absolute
 *    position specified by "position"
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motor has reached the commanded
 *    position)
 *
 * Parameters:
 *    int position  - The absolute position, in step pulses, to move to
 *
 * Returns: True/False depending on whether the move was successfully triggered.
 */
bool XMoveAbsolutePosition(int positionDMM) {
	int position = round(positionDMM * X_STEPS_PER_DMM);  // round to nearest count cause pi
	// Check if a motor alert is currently preventing motion
	// Clear alert if configured to do so
	if (motorFaultPresent()) {
		Serial.println("X Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsX();
		} else {
			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("X Move canceled.");
		Serial.println();
		return false;
	}

	Serial.print("X Moving to absolute position: ");
	Serial.println(position);

	// Command the move of absolute distance
	Xaxis.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);

	// Waits for HLFB to assert (signaling the move has successfully completed)
	Serial.println("Moving.. Waiting for HLFB");
	while ((!Xaxis.StepsComplete() || Xaxis.HlfbState() != MotorDriver::HLFB_ASSERTED) && !Xaxis.StatusReg().bit.AlertsPresent) {
		continue;
	}
	// Check if motor alert occurred during move
	// Clear alert if configured to do so
	if (Xaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("X Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsX();
		} else {
			Serial.println("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Motion may not have completed as expected. Proceed with caution.");
		Serial.println();
		return false;
	} else {
		Serial.println("Move Done");
		return true;
	}
}
bool YMoveAbsolutePosition(int positionDMM) {
	int position = positionDMM * Y_STEPS_PER_DMM;
	// Check if a motor alert is currently preventing motion
	// Clear alert if configured to do so
	if (Yaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("Y Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsY();
		} else {
			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Y Move canceled.");
		Serial.println();
		return false;
	}

	Serial.print("Y Moving to absolute position: ");
	Serial.println(position);

	// Command the move of absolute distance
	Yaxis.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);

	// Waits for HLFB to assert (signaling the move has successfully completed)
	Serial.println("Moving.. Waiting for HLFB");
	while ((!Yaxis.StepsComplete() || Yaxis.HlfbState() != MotorDriver::HLFB_ASSERTED) && !Xaxis.StatusReg().bit.AlertsPresent) {
		continue;
	}
	// Check if motor alert occurred during move
	// Clear alert if configured to do so
	if (Yaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("Y Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsY();
		} else {
			Serial.println("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Motion may not have completed as expected. Proceed with caution.");
		Serial.println();
		return false;
	} else {
		Serial.println("Move Done");
		return true;
	}
}
bool ZMoveAbsolutePosition(int positionDMM) {
	int position = positionDMM * Z_STEPS_PER_DMM;
	// Check if a motor alert is currently preventing motion
	// Clear alert if configured to do so
	if (Zaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("Z Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsZ();
		} else {
			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Z Move canceled.");
		Serial.println();
		return false;
	}

	Serial.print("Z Moving to absolute position: ");
	Serial.println(position);

	// Command the move of absolute distance
	Zaxis.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);

	// Waits for HLFB to assert (signaling the move has successfully completed)
	Serial.println("Moving.. Waiting for HLFB");
	while ((!Zaxis.StepsComplete() || Zaxis.HlfbState() != MotorDriver::HLFB_ASSERTED) && !Xaxis.StatusReg().bit.AlertsPresent) {
		continue;
	}
	// Check if motor alert occurred during move
	// Clear alert if configured to do so
	if (Zaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("Z Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsZ();
		} else {
			Serial.println("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Motion may not have completed as expected. Proceed with caution.");
		Serial.println();
		return false;
	} else {
		Serial.println("Move Done");
		return true;
	}
}
//------------------------------------------------------------------------------


/*------------------------------------------------------------------------------
 * PrintAlerts
 *
 *    Prints active alerts.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
void PrintAlerts() {  // handles all motors
	// report status of alerts on X-axis
	Serial.println("X-axis alerts present: ");
	if (Xaxis.AlertReg().bit.MotionCanceledInAlert) {
		Serial.println("   X MotionCanceledInAlert ");
	}
	if (Xaxis.AlertReg().bit.MotionCanceledPositiveLimit) {
		Serial.println("   X MotionCanceledPositiveLimit ");
	}
	if (Xaxis.AlertReg().bit.MotionCanceledNegativeLimit) {
		Serial.println("   X MotionCanceledNegativeLimit ");
	}
	if (Xaxis.AlertReg().bit.MotionCanceledSensorEStop) {
		Serial.println("   X MotionCanceledSensorEStop ");
	}
	if (Xaxis.AlertReg().bit.MotionCanceledMotorDisabled) {
		Serial.println("   X MotionCanceledMotorDisabled ");
	}
	if (Xaxis.AlertReg().bit.MotorFaulted) {
		Serial.println("   X MotorFaulted ");
	}
	// report status of alerts on Y-axis
	Serial.println("Y-axis alerts present: ");
	if (Yaxis.AlertReg().bit.MotionCanceledInAlert) {
		Serial.println("   Y MotionCanceledInAlert ");
	}
	if (Yaxis.AlertReg().bit.MotionCanceledPositiveLimit) {
		Serial.println("   Y MotionCanceledPositiveLimit ");
	}
	if (Yaxis.AlertReg().bit.MotionCanceledNegativeLimit) {
		Serial.println("   Y MotionCanceledNegativeLimit ");
	}
	if (Yaxis.AlertReg().bit.MotionCanceledSensorEStop) {
		Serial.println("   Y MotionCanceledSensorEStop ");
	}
	if (Yaxis.AlertReg().bit.MotionCanceledMotorDisabled) {
		Serial.println("   Y MotionCanceledMotorDisabled ");
	}
	if (Yaxis.AlertReg().bit.MotorFaulted) {
		Serial.println("   Y MotorFaulted ");
	}
	// report status of alerts on Z-axis
	Serial.println("Z-axis alerts present: ");
	if (Zaxis.AlertReg().bit.MotionCanceledInAlert) {
		Serial.println("   Z MotionCanceledInAlert ");
	}
	if (Zaxis.AlertReg().bit.MotionCanceledPositiveLimit) {
		Serial.println("   Z MotionCanceledPositiveLimit ");
	}
	if (Zaxis.AlertReg().bit.MotionCanceledNegativeLimit) {
		Serial.println("   Z MotionCanceledNegativeLimit ");
	}
	if (Zaxis.AlertReg().bit.MotionCanceledSensorEStop) {
		Serial.println("   Z MotionCanceledSensorEStop ");
	}
	if (Zaxis.AlertReg().bit.MotionCanceledMotorDisabled) {
		Serial.println("   Z MotionCanceledMotorDisabled ");
	}
	if (Zaxis.AlertReg().bit.MotorFaulted) {
		Serial.println("   Z MotorFaulted ");
	}
}
//------------------------------------------------------------------------------

// somewhere: if(motorFaultPresent) {MotionState = PAUSED_MOTOR_FAULT;}
bool motorFaultPresent() {
	return Xaxis.StatusReg().bit.AlertsPresent || Yaxis.StatusReg().bit.AlertsPresent || Zaxis.StatusReg().bit.AlertsPresent;
}
// should move this stuff to function for clearing an alert
/*------------------------------------------------------------------------------
 * HandleAlertsX
 *
 *    Clears alerts, including motor faults. 
 *    Faults are cleared by cycling enable to the motor.
 *    Alerts are cleared by clearing the ClearCore alert register directly.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */

void HandleAlertsX() {
	if (Xaxis.AlertReg().bit.MotorFaulted) {
		// if a motor fault is present, clear it by cycling enable
		Serial.println("Faults present. Cycling enable signal to motor to clear faults.");
		Xaxis.EnableRequest(false);
		Delay_ms(10);
		Xaxis.EnableRequest(true);
	}
	// clear alerts
	Serial.println("Clearing alerts.");
	Xaxis.ClearAlerts();
}
/*------------------------------------------------------------------------------
 * HandleAlertsY
 *
 *    Clears alerts, including motor faults. 
 *    Faults are cleared by cycling enable to the motor.
 *    Alerts are cleared by clearing the ClearCore alert register directly.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
void HandleAlertsY() {
	if (Yaxis.AlertReg().bit.MotorFaulted) {
		// if a motor fault is present, clear it by cycling enable
		Serial.println("Faults present. Cycling enable signal to motor to clear faults.");
		Yaxis.EnableRequest(false);
		Delay_ms(10);
		Yaxis.EnableRequest(true);
	}
	// clear alerts
	Serial.println("Clearing alerts.");
	Yaxis.ClearAlerts();
}
/*------------------------------------------------------------------------------
 * HandleAlertsZ
 *
 *    Clears alerts, including motor faults. 
 *    Faults are cleared by cycling enable to the motor.
 *    Alerts are cleared by clearing the ClearCore alert register directly.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
void HandleAlertsZ() {
	if (Zaxis.AlertReg().bit.MotorFaulted) {
		// if a motor fault is present, clear it by cycling enable
		Serial.println("Faults present. Cycling enable signal to motor to clear faults.");
		Zaxis.EnableRequest(false);
		Delay_ms(10);
		Zaxis.EnableRequest(true);
	}
	// clear alerts
	Serial.println("Clearing alerts.");
	Zaxis.ClearAlerts();
}
//------------------------------------------------------------------------------
double readAirPressure() {
	// int adcResult = analogRead(manifoldPressurePin);
	// double inputVoltage = 10.0 * adcResult / ((1 << adcResolution) - 1);
	double inputVoltage = analogRead(manifoldPressurePin, MILLIVOLTS) / 1000.0;
	// reads <2V@0psi, 4.5V@30psi, 7.1V@60psi, 9.8V@100psi
	// double manifoldPressure = map(inputVoltage, 4.50, 7.10, 30.00, 60.00);
	double manifoldPressure = (inputVoltage - 4.50) * (60.0 - 30.0) / (7.10 - 4.50) + 30.0;
	if (inputVoltage < 2) {
		manifoldPressure = 0;
	}
	Serial.print("Manifold pressure: ");
	Serial.print(manifoldPressure, 0);
	Serial.println(" psi");
	return manifoldPressure;
}
// TODO: sometimes it isn't waiting for all motors to finish homing before moving on
// also observed X axis appearing to be enabled first??
void setupMotors() {
	// Sets the input clocking rate. This normal rate is ideal for ClearPath
	// step and direction applications.
	MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
	// Sets all motor connectors into step and direction mode.
	MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
	// Set the motor's HLFB mode to bipolar PWM
	Xaxis.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
	Yaxis.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
	Zaxis.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
	// Set the HFLB carrier frequency to 482 Hz
	Xaxis.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
	Yaxis.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
	Zaxis.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
	// Sets the maximum velocity for each move
	Xaxis.VelMax(xVelLim);
	Yaxis.VelMax(yVelLim);
	Zaxis.VelMax(zVelLim);
	// Set the maximum acceleration for each move
	Xaxis.AccelMax(xAccelLim);
	Yaxis.AccelMax(yAccelLim);
	Zaxis.AccelMax(zAccelLim);
	// Set reasonable values for E-stop deceleration (whichever is higher will apply)
	Xaxis.EStopDecelMax(10000 * 8);
	Yaxis.EStopDecelMax(1000000);
	Zaxis.EStopDecelMax(1000000);
	// reverse default direction on Y&Z axes
	Yaxis.PolarityInvertSDDirection(true);
	Zaxis.PolarityInvertSDDirection(true);

	// home Z axis first so bit can't crash
	homeZ();
	// home Y axis last
	homeY();
	delay(1000);
	// now move to "real" homes
	YMoveDistance(Y_HOME_OFFSET);                // positive direction, blocking
	Yaxis.PositionRefSet(0);                     // zero internal position counter at home position
	ZMoveDistance(Z_HOME_OFFSET - TOOL_OFFSET);  // moves up
	Zaxis.PositionRefSet(0);                     // zero internal position counter at home position
	ZMoveAbsolutePosition(Z_PARK);               // down a little bit to elimate bit scratching on stick
	// enable X axis
	enableX();
}
void enableX() {
	Xaxis.EnableRequest(true);
	Serial.println("X-axis enabled");
	// Waits for HLFB to assert
	Serial.println("Waiting for HLFB...");
	while (Xaxis.HlfbState() != MotorDriver::HLFB_ASSERTED && !Xaxis.StatusReg().bit.AlertsPresent) {
		// continue;
		delay(20);
	}
	// Check if motor alert occurred during enabling
	// Clear alert if configured to do so
	if (Xaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsX();
		} else {
			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Enabling may not have completed as expected. Proceed with caution.");
		Serial.println();
	} else {
		Serial.println("X-axis ready");
	}
}
void homeY() {
	// Enables the motor; homing will begin automatically if enabled
	Yaxis.EnableRequest(true);
	Serial.println("Y-axis enabled");
	// Waits for HLFB to assert (waits for homing to complete if applicable)
	Serial.print("Waiting for HLFB");
	while (Yaxis.HlfbState() != MotorDriver::HLFB_ASSERTED && !Yaxis.StatusReg().bit.AlertsPresent) {
		// continue;
		delay(100);
		Serial.print(".");
	}
	Serial.println();
	// Check if motor alert occurred during enabling
	// Clear alert if configured to do so
	if (Yaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsZ();
		} else {
			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Enabling may not have completed as expected. Proceed with caution.");
		Serial.println();
	} else {
		Serial.println("Y-axis ready");
	}
}
void homeZ() {
	// Enables the motor; homing will begin automatically if enabled
	Zaxis.EnableRequest(true);
	Serial.println("Z-axis enabled");
	// Waits for HLFB to assert (waits for homing to complete if applicable)
	Serial.print("Waiting for HLFB");
	while (Zaxis.HlfbState() != MotorDriver::HLFB_ASSERTED && !Zaxis.StatusReg().bit.AlertsPresent) {
		// continue; // ??
		delay(100);
		Serial.print(".");
	}
	Serial.println();
	// Check if motor alert occurred during enabling
	// Clear alert if configured to do so
	if (Zaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlertsY();
		} else {
			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Enabling may not have completed as expected. Proceed with caution.");
		Serial.println();
	} else {
		Serial.println("Z-axis ready");
	}
}
void setupPins() {
	analogReadResolution(adcResolution);
	// Set up the CCIO-8 COM port.
	CcioPort.Mode(Connector::CCIO);
	CcioPort.PortOpen();
	// Initialize the CCIO-8 board.
	ccioBoardCount = CcioMgr.CcioCount();
	// CCIO_PINS_PER_BOARD is a constant defined in the main ClearCore library
	// that evaluates to 8.
	ccioPinCount = ccioBoardCount * CCIO_PINS_PER_BOARD;
	// Print the number of discovered CCIO-8 boards to the serial port.
	snprintf(msg, MAX_MSG_LEN, "Discovered %d CCIO-8 board", ccioBoardCount);
	Serial.print(msg);

	if (ccioBoardCount != 1) {
		Serial.print("s");
	}
	Serial.println("...");
	Serial.println();
	// TODO: send error message if incorrect number of boards is discovered
	// Set each CCIO-8 pin to the correct mode. The CCIO-8 pins default to
	// being an input so the pin mode doesn't need to be set for input mode.
	for (int i = 0; i < sizeof(clampPins) / sizeof(clampPins[0]); i++) {
		pinMode(clampPins[i], OUTPUT);
	}
	// pinMode(clamp1Pin, OUTPUT);
	// pinMode(clamp2Pin, OUTPUT);
	// pinMode(clamp3Pin, OUTPUT);
	// pinMode(clamp4Pin, OUTPUT);
	// pinMode(clamp5Pin, OUTPUT);
	// pinMode(clamp6Pin, OUTPUT);
	// pinMode(clamp7Pin, OUTPUT);
	// pinMode(clamp8Pin, OUTPUT);
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

	// // test pneumatics wiring
	// for (int pin = CLEARCORE_PIN_CCIOB0; pin <= CLEARCORE_PIN_CCIOB7; pin++) {
	//   digitalWrite(pin, HIGH);
	// 	delay(1000);
	// }
	// for (int pin = CLEARCORE_PIN_CCIOB0; pin <= CLEARCORE_PIN_CCIOB7; pin++) {
	//   digitalWrite(pin, LOW);
	// 	delay(1000);
	// }
	// test laser beams TODO: make more efficient and discriminate error printing
	bool laserError = 0;
	if (!digitalRead(beam1Pin)) { laserError = 1; }
	digitalWrite(laser1Pin, HIGH);
	delay(100);
	if (digitalRead(beam1Pin)) { laserError = 1; }
	digitalWrite(laser1Pin, LOW);

	if (!digitalRead(beam2Pin)) { laserError = 1; }
	digitalWrite(laser2Pin, HIGH);
	delay(100);
	if (digitalRead(beam2Pin)) { laserError = 1; }
	digitalWrite(laser2Pin, LOW);

	if (!digitalRead(beam3Pin)) { laserError = 1; }
	digitalWrite(laser3Pin, HIGH);
	delay(100);
	if (digitalRead(beam3Pin)) { laserError = 1; }
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
