#pragma once

#include "ClearCore.h"
#include "LPM_axes.h"

enum class MotionPollResult : uint8_t { InProgress, Complete, Faulted };

inline MotionPollResult pollAxisMoveComplete(MotorDriver& axis) {
	if (axis.StatusReg().bit.AlertsPresent) {
		return MotionPollResult::Faulted;
	}
	if (axis.StepsComplete() && axis.HlfbState() == MotorDriver::HLFB_ASSERTED) {
		return MotionPollResult::Complete;
	}
	return MotionPollResult::InProgress;
}

inline bool axisHasAlert(MotorDriver& axis) {
	return axis.StatusReg().bit.AlertsPresent;
}

bool motorFaultPresent();

inline bool XMoveAbsoluteStartCounts(int32_t positionCounts) {
	if (motorFaultPresent()) {
		return false;
	}
	if (Xaxis.StatusReg().bit.AlertsPresent) {
		return false;
	}
	Xaxis.Move(positionCounts, MotorDriver::MOVE_TARGET_ABSOLUTE);
	return true;
}

inline bool YMoveAbsoluteStartCounts(int32_t positionCounts) {
	if (motorFaultPresent()) {
		return false;
	}
	if (Yaxis.StatusReg().bit.AlertsPresent) {
		return false;
	}
	Yaxis.Move(positionCounts, MotorDriver::MOVE_TARGET_ABSOLUTE);
	return true;
}

inline bool ZMoveAbsoluteStartCounts(int32_t positionCounts) {
	if (motorFaultPresent()) {
		return false;
	}
	if (Zaxis.StatusReg().bit.AlertsPresent) {
		return false;
	}
	Zaxis.Move(positionCounts, MotorDriver::MOVE_TARGET_ABSOLUTE);
	return true;
}

inline bool XMoveDistanceStartCounts(int32_t distanceCounts) {
	if (motorFaultPresent()) {
		return false;
	}
	if (Xaxis.StatusReg().bit.AlertsPresent) {
		return false;
	}
	Xaxis.Move(distanceCounts);
	return true;
}

inline bool ZMoveDistanceStartCounts(int32_t distanceCounts) {
	if (motorFaultPresent()) {
		return false;
	}
	if (Zaxis.StatusReg().bit.AlertsPresent) {
		return false;
	}
	Zaxis.Move(distanceCounts);
	return true;
}

inline bool YMoveDistanceStartCounts(int32_t distanceCounts) {
	if (motorFaultPresent()) {
		return false;
	}
	if (Yaxis.StatusReg().bit.AlertsPresent) {
		return false;
	}
	Yaxis.Move(distanceCounts);
	return true;
}
