// simple sketch to test the accuracy/precision of locating system (no clamps)

#include "ClearCore.h"
#define Xaxis ConnectorM0  // TODO change to unique name for wheels

// Specify which ClearCore serial COM port is connected to the "COM IN" port
// of the CCIO-8 board. COM-1 may also be used.
#define CcioPort ConnectorCOM0
#define baudRate 115200
#define EstopPin CLEARCORE_PIN_A9 // connected to actual, normally closed, active low circuit
#define power4hubPin CLEARCORE_PIN_IO1 // switches aux power to hub through relay, to allow forcing axes to home on startup#define laser2Pin CLEARCORE_PIN_CCIOC1
#define laser2Pin CLEARCORE_PIN_CCIOC1
#define beam2Pin CLEARCORE_PIN_CCIOC6

// This example has built-in functionality to automatically clear motor alerts,
//	including motor shutdowns. Any uncleared alert will cancel and disallow motion.
// WARNING: enabling automatic alert handling will clear alerts immediately when
//	encountered and return a motor to a state in which motion is allowed. Before
//	enabling this functionality, be sure to understand this behavior and ensure
//	your system will not enter an unsafe state.
// To enable automatic alert handling, #define HANDLE_ALERTS (1)
// To disable automatic alert handling, #define HANDLE_ALERTS (0)
#define HANDLE_ALERTS (1)

// Define the velocity and acceleration limits to be used for each move
int velocityLimit = 2000 * 8;       // pulses per sec
int accelerationLimit = 30000;  // pulses per sec^2

// Declares user-defined helper functions.
// The definition/implementations of these functions are at the bottom of the sketch.
// bool MoveAtVelocity(int32_t velocity);
// bool MoveDistance(int distance);
// bool MoveAbsolutePosition(int position);
bool XMoveAtVelocity(int32_t velocity);
bool XMoveDistance(int distance);
bool XMoveAbsolutePosition(int position);
void PrintAlerts();
void HandleAlerts();
void homeStick();
#define DIAL_INDICATOR 1800
void setup() {
	// Put your setup code here, it will only run once:

	// Sets the input clocking rate. This normal rate is ideal for ClearPath
	// step and direction applications.
	MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

	// Sets all motor connectors into step and direction mode.
	MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
	                      Connector::CPM_MODE_STEP_AND_DIR);

	// Set the motor's HLFB mode to bipolar PWM
	Xaxis.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
	// Set the HFLB carrier frequency to 482 Hz
	Xaxis.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

	// Sets the maximum velocity for each move
	Xaxis.VelMax(velocityLimit);

	// Set the maximum acceleration for each move
	Xaxis.AccelMax(accelerationLimit);
	Xaxis.EStopDecelMax(10000*8);

	// Xaxis.EStopConnector(EstopPin);

	// Set up the CCIO-8 COM port.
	CcioPort.Mode(Connector::CCIO);
	CcioPort.PortOpen();
	pinMode(laser2Pin, OUTPUT);
	pinMode(beam2Pin, INPUT);
	delay(100);
	digitalWrite(laser2Pin, HIGH);
	pinMode(power4hubPin, OUTPUT);
	digitalWrite(power4hubPin, HIGH); // powers up motor
	delay(2000);

	// // Set each CCIO-8 pin to the correct mode. The CCIO-8 pins default to
	//     // being an input so the pin mode doesn't need to be set for input mode.
	//     for (uint8_t ccioPinIndex = 0; ccioPinIndex < ccioPinCount; ccioPinIndex++) {
	//         pinMode(CLEARCORE_PIN_CCIOA0 + ccioPinIndex, OUTPUT);
	//     }

	// Sets up serial communication and waits up to 5 seconds for a port to open.
	// Serial communication is not required for this example to run.
	Serial.begin(baudRate);
	uint32_t timeout = 5000;
	uint32_t startTime = millis();
	while (!Serial && millis() - startTime < timeout) {
		continue;
	}

	// Enables the motor; homing will begin automatically if enabled
	Xaxis.EnableRequest(true);
	Serial.println("Motor Enabled");

	// Waits for HLFB to assert (waits for homing to complete if applicable)
	Serial.println("Waiting for HLFB...");
	while (Xaxis.HlfbState() != MotorDriver::HLFB_ASSERTED && !Xaxis.StatusReg().bit.AlertsPresent) {
		continue;
	}
	// Check if motor alert occurred during enabling
	// Clear alert if configured to do so
	if (Xaxis.StatusReg().bit.AlertsPresent) {
		Serial.println("Motor alert detected.");
		PrintAlerts();
		if (HANDLE_ALERTS) {
			HandleAlerts();
		} else {
			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Enabling may not have completed as expected. Proceed with caution.");
		Serial.println();
	} else {
		Serial.println("Motor Ready");
	}

	// delay(5000);
	// while(true); // don't go on to loop for now
}

void loop() {

	Serial.println("loop start");
	homeStick();
	XMoveAbsolutePosition((DIAL_INDICATOR-20)*8);
	Xaxis.VelMax(10*8);
	XMoveAbsolutePosition(DIAL_INDICATOR*8);
	Xaxis.VelMax(velocityLimit);  // return to previous value
	delay(10000);
	XMoveDistance(-1850*8);

	// MoveDistance(-400);
	// delay(1000);
	// MoveDistance(-400);
	// delay(1000);
	// MoveDistance(400);
	// delay(1000);
	// MoveDistance(400);
	// delay(3000);
}

void homeStick() {
	if (digitalRead(beam2Pin)) {
		while (digitalRead(beam2Pin)) {
			XMoveAtVelocity(-400*8);
		}
	}
	delay(1000);
	while (!digitalRead(beam2Pin)) {
		XMoveAtVelocity(400*8);
	}
	Xaxis.MoveStopAbrupt();
	delay(200);
	Xaxis.AccelMax(1000*8);  // temporarily bump down for homing
	XMoveDistance(-10*8);
	delay(200);
	while (!digitalRead(beam2Pin)) {
		XMoveAtVelocity(10);
	}
	Xaxis.MoveStopAbrupt();
	delay(1000);
	// read current StepGenerator postition (internal to ClearCore)
	// Xaxis.PositionRefCommanded();
	Xaxis.PositionRefSet(0);            // zero internal position counter
	Xaxis.AccelMax(accelerationLimit);  // return this to original value
}

bool XMoveDistance(int distance) {
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
bool XMoveAbsolutePosition(int position) {
	// int position = round(positionDMM*X_STEPS_PER_DMM); // round to nearest count cause pi
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
bool XMoveAtVelocity(int velocity) {
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

// /*------------------------------------------------------------------------------
//  * MoveAtVelocity
//  *
//  *    Command the motor to move at the specified "velocity", in pulses/second.
//  *    Prints the move status to the USB serial port
//  *
//  * Parameters:
//  *    int velocity  - The velocity, in step pulses/sec, to command
//  *
//  * Returns: True/False depending on whether the move was successfully triggered.
//  */
// bool MoveAtVelocity(int velocity) {
// 	// Check if a motor alert is currently preventing motion
// 	// Clear alert if configured to do so
// 	if (Xaxis.StatusReg().bit.AlertsPresent) {
// 		Serial.println("Motor alert detected.");
// 		PrintAlerts();
// 		if (HANDLE_ALERTS) {
// 			HandleAlerts();
// 		} else {
// 			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
// 		}
// 		Serial.println("Move canceled.");
// 		Serial.println();
// 		return false;
// 	}

// 	Serial.print("Moving at velocity: ");
// 	Serial.println(velocity);

// 	// Command the velocity move
// 	Xaxis.MoveVelocity(velocity);

// 	// Waits for the step command to ramp up/down to the commanded velocity.
// 	// This time will depend on your Acceleration Limit.
// 	Serial.println("Ramping to speed...");
// 	while (!Xaxis.StatusReg().bit.AtTargetVelocity) {
// 		continue;
// 	}

// 	Serial.println("At Speed");
// 	return true;
// }
// //------------------------------------------------------------------------------


// /*------------------------------------------------------------------------------
//  * MoveDistance
//  *
//  *    Command "distance" number of step pulses away from the current position
//  *    Prints the move status to the USB serial port
//  *    Returns when HLFB asserts (indicating the motor has reached the commanded
//  *    position)
//  *
//  * Parameters:
//  *    int distance  - The distance, in step pulses, to move
//  *
//  * Returns: True/False depending on whether the move was successfully triggered.
//  */
// bool MoveDistance(int distance) {
// 	// Check if a motor alert is currently preventing motion
// 	// Clear alert if configured to do so
// 	if (Xaxis.StatusReg().bit.AlertsPresent) {
// 		Serial.println("Motor alert detected.");
// 		PrintAlerts();
// 		if (HANDLE_ALERTS) {
// 			HandleAlerts();
// 		} else {
// 			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
// 		}
// 		Serial.println("Move canceled.");
// 		Serial.println();
// 		return false;
// 	}

// 	Serial.print("Moving distance: ");
// 	Serial.println(distance);

// 	// Command the move of incremental distance
// 	Xaxis.Move(distance);

// 	// Waits for HLFB to assert (signaling the move has successfully completed)
// 	Serial.println("Moving.. Waiting for HLFB");
// 	while ((!Xaxis.StepsComplete() || Xaxis.HlfbState() != MotorDriver::HLFB_ASSERTED) && !Xaxis.StatusReg().bit.AlertsPresent) {
// 		continue;
// 	}
// 	// Check if motor alert occurred during move
// 	// Clear alert if configured to do so
// 	if (Xaxis.StatusReg().bit.AlertsPresent) {
// 		Serial.println("Motor alert detected.");
// 		PrintAlerts();
// 		if (HANDLE_ALERTS) {
// 			HandleAlerts();
// 		} else {
// 			Serial.println("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
// 		}
// 		Serial.println("Motion may not have completed as expected. Proceed with caution.");
// 		Serial.println();
// 		return false;
// 	} else {
// 		Serial.println("Move Done");
// 		return true;
// 	}
// }
// //------------------------------------------------------------------------------


// /*------------------------------------------------------------------------------
//  * MoveAbsolutePosition
//  *
//  *    Command step pulses to move the motor's current position to the absolute
//  *    position specified by "position"
//  *    Prints the move status to the USB serial port
//  *    Returns when HLFB asserts (indicating the motor has reached the commanded
//  *    position)
//  *
//  * Parameters:
//  *    int position  - The absolute position, in step pulses, to move to
//  *
//  * Returns: True/False depending on whether the move was successfully triggered.
//  */
// bool MoveAbsolutePosition(int position) {
// 	// Check if a motor alert is currently preventing motion
// 	// Clear alert if configured to do so
// 	if (Xaxis.StatusReg().bit.AlertsPresent) {
// 		Serial.println("Motor alert detected.");
// 		PrintAlerts();
// 		if (HANDLE_ALERTS) {
// 			HandleAlerts();
// 		} else {
// 			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
// 		}
// 		Serial.println("Move canceled.");
// 		Serial.println();
// 		return false;
// 	}

// 	Serial.print("Moving to absolute position: ");
// 	Serial.println(position);

// 	// Command the move of absolute distance
// 	Xaxis.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);

// 	// Waits for HLFB to assert (signaling the move has successfully completed)
// 	Serial.println("Moving.. Waiting for HLFB");
// 	while ((!Xaxis.StepsComplete() || Xaxis.HlfbState() != MotorDriver::HLFB_ASSERTED) && !Xaxis.StatusReg().bit.AlertsPresent) {
// 		continue;
// 	}
// 	// Check if motor alert occurred during move
// 	// Clear alert if configured to do so
// 	if (Xaxis.StatusReg().bit.AlertsPresent) {
// 		Serial.println("Motor alert detected.");
// 		PrintAlerts();
// 		if (HANDLE_ALERTS) {
// 			HandleAlerts();
// 		} else {
// 			Serial.println("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
// 		}
// 		Serial.println("Motion may not have completed as expected. Proceed with caution.");
// 		Serial.println();
// 		return false;
// 	} else {
// 		Serial.println("Move Done");
// 		return true;
// 	}
// }
// //------------------------------------------------------------------------------


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
void PrintAlerts() {
	// report status of alerts
	Serial.println("Alerts present: ");
	if (Xaxis.AlertReg().bit.MotionCanceledInAlert) {
		Serial.println("    MotionCanceledInAlert ");
	}
	if (Xaxis.AlertReg().bit.MotionCanceledPositiveLimit) {
		Serial.println("    MotionCanceledPositiveLimit ");
	}
	if (Xaxis.AlertReg().bit.MotionCanceledNegativeLimit) {
		Serial.println("    MotionCanceledNegativeLimit ");
	}
	if (Xaxis.AlertReg().bit.MotionCanceledSensorEStop) {
		Serial.println("    MotionCanceledSensorEStop ");
	}
	if (Xaxis.AlertReg().bit.MotionCanceledMotorDisabled) {
		Serial.println("    MotionCanceledMotorDisabled ");
	}
	if (Xaxis.AlertReg().bit.MotorFaulted) {
		Serial.println("    MotorFaulted ");
	}
}
//------------------------------------------------------------------------------


/*------------------------------------------------------------------------------
 * HandleAlerts
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
void HandleAlerts() {
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
//------------------------------------------------------------------------------
