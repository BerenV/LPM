#pragma once

#include <Arduino.h>

enum class MotionState : uint8_t {
	IDLE,
	MOVING,
	PAUSED_ESTOP,
	RECOVERING_ESTOP,
	PAUSED_PRESSURE,
	PAUSED_MOTOR_FAULT
};

extern MotionState motionState;

inline const char* motionStateName(MotionState s) {
	switch (s) {
	case MotionState::IDLE:
		return "IDLE";
	case MotionState::MOVING:
		return "MOVING";
	case MotionState::PAUSED_ESTOP:
		return "PAUSED_ESTOP";
	case MotionState::RECOVERING_ESTOP:
		return "RECOVERING_ESTOP";
	case MotionState::PAUSED_PRESSURE:
		return "PAUSED_PRESSURE";
	case MotionState::PAUSED_MOTOR_FAULT:
		return "PAUSED_MOTOR_FAULT";
	}
	return "?";
}

void machineInit();
void supervisorTick();
void machineTick();

void clampManager();
