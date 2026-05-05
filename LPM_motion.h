#pragma once

#include "ClearCore.h"
#include "LPM_axes.h"

enum class MotionPollResult : uint8_t { InProgress, Complete, Faulted };

inline bool axisMotionFault(MotorDriver& m) {
	return m.StatusReg().bit.AlertsPresent || m.AlertReg().bit.MotorFaulted;
}

inline MotionPollResult pollAxisMoveComplete(MotorDriver& axis) {
	if (axisMotionFault(axis)) {
		return MotionPollResult::Faulted;
	}
	if (axis.StepsComplete() && axis.HlfbState() == MotorDriver::HLFB_ASSERTED) {
		return MotionPollResult::Complete;
	}
	return MotionPollResult::InProgress;
}

inline bool axisHasAlert(MotorDriver& axis) {
	return axisMotionFault(axis);
}

bool motorFaultPresent();

inline bool XMoveAbsoluteStartCounts(int32_t positionCounts) {
	if (motorFaultPresent()) {
		return false;
	}
	if (axisMotionFault(Xaxis)) {
		return false;
	}
	Xaxis.Move(positionCounts, MotorDriver::MOVE_TARGET_ABSOLUTE);
	return true;
}

inline bool YMoveAbsoluteStartCounts(int32_t positionCounts) {
	if (motorFaultPresent()) {
		return false;
	}
	if (axisMotionFault(Yaxis)) {
		return false;
	}
	Yaxis.Move(positionCounts, MotorDriver::MOVE_TARGET_ABSOLUTE);
	return true;
}

inline bool ZMoveAbsoluteStartCounts(int32_t positionCounts) {
	if (motorFaultPresent()) {
		return false;
	}
	if (axisMotionFault(Zaxis)) {
		return false;
	}
	Zaxis.Move(positionCounts, MotorDriver::MOVE_TARGET_ABSOLUTE);
	return true;
}

inline bool XMoveDistanceStartCounts(int32_t distanceCounts) {
	if (motorFaultPresent()) {
		return false;
	}
	if (axisMotionFault(Xaxis)) {
		return false;
	}
	Xaxis.Move(distanceCounts);
	return true;
}

inline bool ZMoveDistanceStartCounts(int32_t distanceCounts) {
	if (motorFaultPresent()) {
		return false;
	}
	if (axisMotionFault(Zaxis)) {
		return false;
	}
	Zaxis.Move(distanceCounts);
	return true;
}

inline bool YMoveDistanceStartCounts(int32_t distanceCounts) {
	if (motorFaultPresent()) {
		return false;
	}
	if (axisMotionFault(Yaxis)) {
		return false;
	}
	Yaxis.Move(distanceCounts);
	return true;
}
