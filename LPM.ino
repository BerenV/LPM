#include "ClearCore.h"
#include <SPI.h>
#include <SD.h>

#include "LPM_board.h"
#include "LPM_config.h"
#include "LPM_motion.h"
#include "LPM_sd_config.h"
#include "LPM_machine.h"

File configSD;

#define baudRate 115200

// SASH OFFSETS (everything in counts) — tune in LPM_config / SD; notes:
// entering wheels to laser 1: 2848
// laser 1 to tip at spindle center: 6837
// tip at spindle center to laser 3: 1855
// laser 3 to laser 2: 647
// clamp 2 can come down as soon as tip is past spindle
// clamp 3 down as soon as tip is 3315 counts past spindle (right before third wheels grip)
// other end of stick at laser 1 to spindle center: 9111
// other end of stick at laser 1 to clamp 2 raising: 5623
// other end of stick at spindle center to beam 2: -450
// clamp 3 needs to raise as soon as tip is past spindle
// beam 2 to beam 3: 684
// beam 3 to stick leaving wheels: 9202


// These will be used to format the text that is printed to the serial port.
#define MAX_MSG_LEN 80
char msg[MAX_MSG_LEN + 1];

// Define the initial velocity and acceleration limits
int xVelLim = 2000*8;     // pulses per sec
int xAccelLim = 10000*8;  // pulses per sec^2
int yVelLim = 15000;     // pulses per sec
int yAccelLim = 1000000;  // pulses per sec^2
int zVelLim = 15000;     // pulses per sec
int zAccelLim = 1000000;  // pulses per sec^2
bool dir = 0; // for spindle test
double manifoldPressure = 0.0; // psi

// Declares user-defined helper functions (prototypes).
void startSpindle();
void stopSpindle();
void PrintAlerts();
void HandleAlertsX();
void HandleAlertsY();
void HandleAlertsZ();
void setupMotors();
void enableX();
void homeY();
void homeZ();
double readAirPressure();

void setup() {
	// Sets up serial communication and waits up to "timeout" seconds for a port to open.
	Serial.begin(baudRate);
	// uint32_t timeout = 1000;
	// uint32_t startTime = millis();
	// while (!Serial && millis() - startTime < timeout) {
	// 	continue;
	// }
	Serial.println();

	setupPins();
	tryLoadSdConfig();
	digitalWrite(power4hubPin, LOW); // do this to force motors to reset and home upon reboot
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
	delay(2000); // allow time for IPC to power up
	setupMotors();
	digitalWrite(yellowMastPin, LOW);
	digitalWrite(greenMastPin, HIGH);  // turns on green "power" light

	machineInit();

	readAirPressure();

}
// fault behavior: should pause, turn off green mast light, start flashing red light until e-stop is (pressed) and released. Then cut red light, turn yellow light on, re-enable motors, wait for them to stop moving to their last commanded positions, yellow off, and green back on.
// want to 

void loop() {
	supervisorTick();
	machineTick();

	static uint32_t lastPressureSampleMs = 0;
	uint32_t now = millis();
	if (lastPressureSampleMs == 0 || (int32_t)(now - lastPressureSampleMs) >= 1000) {
		lastPressureSampleMs = now;
		readAirPressure();
		Serial.print("status motion=");
		Serial.print(motionStateName(motionState));
		Serial.print(" pressure_psi=");
		Serial.println(manifoldPressure, 1);
	}
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

// Same poll primitive as LPM_machine.cpp; only used while setup() runs.
static bool waitAxisMoveComplete(MotorDriver& axis) {
	for (;;) {
		MotionPollResult r = pollAxisMoveComplete(axis);
		if (r == MotionPollResult::Faulted) {
			PrintAlerts();
			return false;
		}
		if (r == MotionPollResult::Complete) {
			return true;
		}
		delay(1);
	}
}

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
void PrintAlerts() { // handles all motors
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

/*------------------------------------------------------------------------------
 * HandleAlertsZ
 *
 *    Checks status registers for present alerts
 *
 * Parameters:
 *    none
 *
 * Returns: 
 *    True if any motors have a fault
 */
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
/*------------------------------------------------------------------------------
 * readAirPressure
 *
 *    Polls ADC for voltage, converts to pressure in psi
 *		Updates manifoldPressure variable
 *
 * Parameters:
 *    none
 *
 * Returns: 
 *    Manifold pressure in psi
 */
double readAirPressure() {
	// int adcResult = analogRead(manifoldPressurePin);
	// double inputVoltage = 10.0 * adcResult / ((1 << adcResolution) - 1);
	double inputVoltage = analogRead(manifoldPressurePin, MILLIVOLTS) / 1000.0;
	// reads <2V@0psi, 4.5V@30psi, 7.1V@60psi, 9.8V@100psi
	// double manifoldPressure = map(inputVoltage, 4.50, 7.10, 30.00, 60.00);
	manifoldPressure = (inputVoltage - 4.50) * (60.0 - 30.0) / (7.10 - 4.50) + 30.0;
	if (inputVoltage < 2) {
		manifoldPressure = 0;
	}
	// Serial.print("Manifold pressure: ");
	// Serial.print(manifoldPressure, 1);
	// Serial.println(" psi");
	return manifoldPressure;
}
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
	Xaxis.EStopDecelMax(10000*8);
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
	// Move to application homes — offsets Y_HOME/Z_HOME/TOOL are stored as counts; Z_PARK is DMM.
	{
		if (YMoveDistanceStartCounts((int32_t)Y_HOME_OFFSET_COUNTS)) {
			waitAxisMoveComplete(Yaxis);
		}
		Yaxis.PositionRefSet(0);

		int32_t zDeltaCounts = (int32_t)Z_HOME_OFFSET_COUNTS - (int32_t)TOOL_OFFSET_COUNTS;
		if (ZMoveDistanceStartCounts(zDeltaCounts)) {
			waitAxisMoveComplete(Zaxis);
		}
		Zaxis.PositionRefSet(0);

		int32_t zParkCounts = (int32_t)(Z_PARK * Z_STEPS_PER_DMM);
		if (ZMoveAbsoluteStartCounts(zParkCounts)) {
			waitAxisMoveComplete(Zaxis);
		}
	}
	// enable X axis
	enableX();
}

void enableX() {
	Xaxis.EnableRequest(true);
	Serial.println("X-axis enabled");
	Serial.println("Waiting for HLFB...");
	uint32_t t0 = millis();
	while (Xaxis.HlfbState() != MotorDriver::HLFB_ASSERTED && !axisMotionFault(Xaxis)) {
		if (LPM_X_ENABLE_TIMEOUT_MS > 0 && (millis() - t0 >= (uint32_t)LPM_X_ENABLE_TIMEOUT_MS)) {
			digitalWrite(greenMastPin, LOW);
			Serial.println("FAULT: X HLFB never asserted (motor unplugged, no power, or HLFB wiring).");
			while (true) {
				delay(2000);
				Serial.println("Fix X motor connection, then reset the controller.");
			}
		}
		delay(20);
	}
	if (axisMotionFault(Xaxis)) {
		digitalWrite(greenMastPin, LOW);
		Serial.println("Motor fault on X during enable.");
		PrintAlerts();
		while (true) {
			delay(2000);
			Serial.println("Clear the fault at the drive, then reset.");
		}
	}
	Serial.println("X-axis ready");
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
		// if (HANDLE_ALERTS) {
		// 	HandleAlertsZ();
		// } else {
		// 	Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		// }
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
		// if (HANDLE_ALERTS) {
		// 	HandleAlertsY();
		// } else {
		// 	Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		// }
		Serial.println("Enabling may not have completed as expected. Proceed with caution.");
		Serial.println();
	} else {
		Serial.println("Z-axis ready");
	}
}