#pragma once

#include <Arduino.h>

enum class MotionState : uint8_t {
	IDLE,
	MOVING,
	PAUSED_ESTOP,
	PAUSED_PRESSURE,
	PAUSED_MOTOR_FAULT
};

extern MotionState motionState;

void machineInit();
void supervisorTick();
void machineTick();

void clampManager();
