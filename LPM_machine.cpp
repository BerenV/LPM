#include "LPM_machine.h"

#include "LPM_axes.h"
#include "LPM_board.h"
#include "LPM_config.h"
#include "LPM_motion.h"

#include <cmath>
#include <cstdio>

MotionState motionState = MotionState::IDLE;

extern int xVelLim;
extern int yVelLim;
extern int zVelLim;
extern int xAccelLim;
extern int yAccelLim;
extern int zAccelLim;

extern void startSpindle();
extern void stopSpindle();
extern double readAirPressure();
extern void PrintAlerts();

extern double manifoldPressure;

enum class MachinePhase : uint8_t {
	CommissionFeedX,
	RoughSetRef,
	ClampingRun,
	PrecisionHome,
	DrillForward,
	AfterDrillMoveX,
	PrecisionRevHome,
	DrillReverse,
	EjectSeek,
	EjectDistance,
	CycleDoneWait,
};

static MachinePhase phase = MachinePhase::CommissionFeedX;

// --- Commissioning / beam1 ---
bool beam1Initial = false;
uint32_t debounceStableMs = 0;
bool beam1EventArmed = false;

// --- Clamping run ---
bool sawBeam2Clear = false;
uint32_t beam2LowSinceMs = 0;

// --- Precision homing (beam2 + beam3) ---
enum class Ph23 : uint8_t {
	RetreatIfBeam2High,
	ForwardFastUntilBeam2Break,
	StopAfterFast,
	DelayAfterStop1,
	NudgeBack,
	DelayAfterNudge,
	CreepAlignBeam2,
	StopAfterCreep,
	DelayBeforeBeam3,
	Beam3EdgeSeek,
	CheckBeam23Spacing,
	SetSashRef,
	Done
};

Ph23 ph23 = Ph23::RetreatIfBeam2High;
uint32_t waitUntilMs = 0;
int32_t xAtBeam3Event = 0;
int32_t xBeforeBeam3Jog = 0;
bool beam3Initial = false;
bool moveIssued = false;

// --- Drill forward / reverse (hole coordinates from original sketch) ---
enum class DrillFwd : uint8_t {
	Idle,
	StartSpindleWait,
	ZPark,
	Y1,
	XHole1,
	Peck1a,
	Peck1b,
	Peck1c,
	XHole2,
	Peck2a,
	Peck2b,
	Peck2c,
	ZMid,
	Y2,
	XHole3,
	Peck3a,
	Peck3b,
	Peck3c,
	XHole4,
	Peck4a,
	Peck4b,
	Peck4c,
	ZEnd,
	StopSpindleWait,
	Done
};

enum class DrillRev : uint8_t {
	Idle,
	StartSpindleWait,
	ZPark,
	Y1,
	XHole1,
	Peck1a,
	Peck1b,
	Peck1c,
	XHole2,
	Peck2a,
	Peck2b,
	Peck2c,
	ZMid,
	Y2,
	XHole3,
	Peck3a,
	Peck3b,
	Peck3c,
	XHole4,
	Peck4a,
	Peck4b,
	Peck4c,
	ZEnd,
	StopSpindleWait,
	Done
};

DrillFwd drillFwd = DrillFwd::Idle;
DrillRev drillRev = DrillRev::Idle;

const int Yoffset1 = 240;
const int Zstart1 = 300;
const int Zstop1 = 400;
const int Yoffset2 = 480;
const int Zstart2 = 230;
const int Zstop2 = 300;
const int hole1Fwd = 570;
const int hole2Fwd = 1070;
const int hole3Fwd = 1070;
const int hole4Fwd = 470;
const int hole1Rev = -570;
const int hole2Rev = -1070;
const int hole3Rev = -1070;
const int hole4Rev = -470;

uint32_t spindleWaitUntil = 0;

enum class PhRev : uint8_t {
	ForwardFastWhileBeam2High,
	DelayAfterFast,
	NudgeBack,
	DelayAfterNudge,
	CreepWhileBeam2High,
	DelayFinal,
	Done
};

static PhRev phRev = PhRev::ForwardFastWhileBeam2High;

static bool motionAllowed() {
	return motionState == MotionState::IDLE || motionState == MotionState::MOVING;
}

static bool bypassPressureForBeam1Commissioning() {
#if !LPM_ALLOW_LOW_AIR_DURING_BEAM1_FEED
	return false;
#else
	return phase == MachinePhase::CommissionFeedX || phase == MachinePhase::ClampingRun
	       || phase == MachinePhase::RoughSetRef;
#endif
}

static void stopAllAxesSmooth() {
	Xaxis.MoveStopAbrupt();
	Yaxis.MoveStopAbrupt();
	Zaxis.MoveStopAbrupt();
}

static bool allAxesHlfbAsserted() {
	if (axisMotionFault(Xaxis) || axisMotionFault(Yaxis) || axisMotionFault(Zaxis)) {
		return false;
	}
	return Xaxis.HlfbState() == MotorDriver::HLFB_ASSERTED
	       && Yaxis.HlfbState() == MotorDriver::HLFB_ASSERTED
	       && Zaxis.HlfbState() == MotorDriver::HLFB_ASSERTED;
}

static bool debouncedBeamChange(bool current, bool initial, uint32_t stableNeededMs) {
	if (current != initial) {
		if (debounceStableMs == 0) {
			debounceStableMs = millis();
		}
		return (millis() - debounceStableMs) >= stableNeededMs;
	}
	debounceStableMs = 0;
	return false;
}

static int32_t xCountsFromDmm(int dmm) {
	return (int32_t)lround((double)dmm * X_STEPS_PER_DMM);
}

static int32_t yCountsFromDmm(int dmm) {
	return (int32_t)(dmm * Y_STEPS_PER_DMM);
}

static int32_t zCountsFromDmm(int dmm) {
	return (int32_t)(dmm * Z_STEPS_PER_DMM);
}

static bool pollMove(MotorDriver& axis) {
	MotionPollResult r = pollAxisMoveComplete(axis);
	if (r == MotionPollResult::Faulted) {
		PrintAlerts();
		motionState = MotionState::PAUSED_MOTOR_FAULT;
		return false;
	}
	return r == MotionPollResult::Complete;
}

static void resetDrillFwd() {
	drillFwd = DrillFwd::StartSpindleWait;
	moveIssued = false;
	spindleWaitUntil = 0;
}

static void resetDrillRev() {
	drillRev = DrillRev::StartSpindleWait;
	moveIssued = false;
	spindleWaitUntil = 0;
}

static bool drillFwdTick() {
	switch (drillFwd) {
	case DrillFwd::Idle:
		return true;
	case DrillFwd::StartSpindleWait:
		if (spindleWaitUntil == 0) {
			startSpindle();
			spindleWaitUntil = millis() + 1200;
		}
		if ((int32_t)(millis() - spindleWaitUntil) >= 0) {
			drillFwd = DrillFwd::ZPark;
			moveIssued = false;
			spindleWaitUntil = 0;
		}
		return false;
	case DrillFwd::ZPark:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Z_PARK));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::Y1;
			moveIssued = false;
		}
		return false;
	case DrillFwd::Y1:
		if (!moveIssued) {
			moveIssued = YMoveAbsoluteStartCounts(yCountsFromDmm(Yoffset1));
		}
		if (moveIssued && pollMove(Yaxis)) {
			drillFwd = DrillFwd::XHole1;
			moveIssued = false;
		}
		return false;
	case DrillFwd::XHole1:
		if (!moveIssued) {
			moveIssued = XMoveAbsoluteStartCounts(xCountsFromDmm(hole1Fwd));
		}
		if (moveIssued && pollMove(Xaxis)) {
			drillFwd = DrillFwd::Peck1a;
			moveIssued = false;
		}
		return false;
	case DrillFwd::Peck1a:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart1));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::Peck1b;
			moveIssued = false;
			Zaxis.VelMax(DRILL_Z_VELOCITY);
		}
		return false;
	case DrillFwd::Peck1b:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstop1));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::Peck1c;
			moveIssued = false;
			Zaxis.VelMax(zVelLim);
		}
		return false;
	case DrillFwd::Peck1c:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart1));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::XHole2;
			moveIssued = false;
		}
		return false;
	case DrillFwd::XHole2:
		if (!moveIssued) {
			moveIssued = XMoveAbsoluteStartCounts(xCountsFromDmm(hole2Fwd));
		}
		if (moveIssued && pollMove(Xaxis)) {
			drillFwd = DrillFwd::Peck2a;
			moveIssued = false;
		}
		return false;
	case DrillFwd::Peck2a:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart1));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::Peck2b;
			moveIssued = false;
			Zaxis.VelMax(DRILL_Z_VELOCITY);
		}
		return false;
	case DrillFwd::Peck2b:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstop1));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::Peck2c;
			moveIssued = false;
			Zaxis.VelMax(zVelLim);
		}
		return false;
	case DrillFwd::Peck2c:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart1));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::ZMid;
			moveIssued = false;
		}
		return false;
	case DrillFwd::ZMid:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(180));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::Y2;
			moveIssued = false;
		}
		return false;
	case DrillFwd::Y2:
		if (!moveIssued) {
			moveIssued = YMoveAbsoluteStartCounts(yCountsFromDmm(Yoffset2));
		}
		if (moveIssued && pollMove(Yaxis)) {
			drillFwd = DrillFwd::XHole3;
			moveIssued = false;
		}
		return false;
	case DrillFwd::XHole3:
		if (!moveIssued) {
			moveIssued = XMoveAbsoluteStartCounts(xCountsFromDmm(hole3Fwd));
		}
		if (moveIssued && pollMove(Xaxis)) {
			drillFwd = DrillFwd::Peck3a;
			moveIssued = false;
		}
		return false;
	case DrillFwd::Peck3a:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart2));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::Peck3b;
			moveIssued = false;
			Zaxis.VelMax(DRILL_Z_VELOCITY);
		}
		return false;
	case DrillFwd::Peck3b:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstop2));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::Peck3c;
			moveIssued = false;
			Zaxis.VelMax(zVelLim);
		}
		return false;
	case DrillFwd::Peck3c:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart2));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::XHole4;
			moveIssued = false;
		}
		return false;
	case DrillFwd::XHole4:
		if (!moveIssued) {
			moveIssued = XMoveAbsoluteStartCounts(xCountsFromDmm(hole4Fwd));
		}
		if (moveIssued && pollMove(Xaxis)) {
			drillFwd = DrillFwd::Peck4a;
			moveIssued = false;
		}
		return false;
	case DrillFwd::Peck4a:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart2));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::Peck4b;
			moveIssued = false;
			Zaxis.VelMax(DRILL_Z_VELOCITY);
		}
		return false;
	case DrillFwd::Peck4b:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstop2));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::Peck4c;
			moveIssued = false;
			Zaxis.VelMax(zVelLim);
		}
		return false;
	case DrillFwd::Peck4c:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart2));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::ZEnd;
			moveIssued = false;
		}
		return false;
	case DrillFwd::ZEnd:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(180));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillFwd = DrillFwd::StopSpindleWait;
			moveIssued = false;
			spindleWaitUntil = 0;
		}
		return false;
	case DrillFwd::StopSpindleWait:
		if (spindleWaitUntil == 0) {
			stopSpindle();
			spindleWaitUntil = millis() + 150;
		}
		if ((int32_t)(millis() - spindleWaitUntil) >= 0) {
			drillFwd = DrillFwd::Done;
		}
		return false;
	case DrillFwd::Done:
		return true;
	}
	return true;
}

static bool drillRevTick() {
	switch (drillRev) {
	case DrillRev::Idle:
		return true;
	case DrillRev::StartSpindleWait:
		if (spindleWaitUntil == 0) {
			startSpindle();
			spindleWaitUntil = millis() + 1200;
		}
		if ((int32_t)(millis() - spindleWaitUntil) >= 0) {
			drillRev = DrillRev::ZPark;
			moveIssued = false;
			spindleWaitUntil = 0;
		}
		return false;
	case DrillRev::ZPark:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Z_PARK));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::Y1;
			moveIssued = false;
		}
		return false;
	case DrillRev::Y1:
		if (!moveIssued) {
			moveIssued = YMoveAbsoluteStartCounts(yCountsFromDmm(Yoffset1));
		}
		if (moveIssued && pollMove(Yaxis)) {
			drillRev = DrillRev::XHole1;
			moveIssued = false;
		}
		return false;
	case DrillRev::XHole1:
		if (!moveIssued) {
			moveIssued = XMoveAbsoluteStartCounts(xCountsFromDmm(hole1Rev));
		}
		if (moveIssued && pollMove(Xaxis)) {
			drillRev = DrillRev::Peck1a;
			moveIssued = false;
		}
		return false;
	case DrillRev::Peck1a:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart1));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::Peck1b;
			moveIssued = false;
			Zaxis.VelMax(DRILL_Z_VELOCITY);
		}
		return false;
	case DrillRev::Peck1b:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstop1));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::Peck1c;
			moveIssued = false;
			Zaxis.VelMax(zVelLim);
		}
		return false;
	case DrillRev::Peck1c:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart1));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::XHole2;
			moveIssued = false;
		}
		return false;
	case DrillRev::XHole2:
		if (!moveIssued) {
			moveIssued = XMoveAbsoluteStartCounts(xCountsFromDmm(hole2Rev));
		}
		if (moveIssued && pollMove(Xaxis)) {
			drillRev = DrillRev::Peck2a;
			moveIssued = false;
		}
		return false;
	case DrillRev::Peck2a:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart1));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::Peck2b;
			moveIssued = false;
			Zaxis.VelMax(DRILL_Z_VELOCITY);
		}
		return false;
	case DrillRev::Peck2b:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstop1));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::Peck2c;
			moveIssued = false;
			Zaxis.VelMax(zVelLim);
		}
		return false;
	case DrillRev::Peck2c:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart1));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::ZMid;
			moveIssued = false;
		}
		return false;
	case DrillRev::ZMid:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(180));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::Y2;
			moveIssued = false;
		}
		return false;
	case DrillRev::Y2:
		if (!moveIssued) {
			moveIssued = YMoveAbsoluteStartCounts(yCountsFromDmm(Yoffset2));
		}
		if (moveIssued && pollMove(Yaxis)) {
			drillRev = DrillRev::XHole3;
			moveIssued = false;
		}
		return false;
	case DrillRev::XHole3:
		if (!moveIssued) {
			moveIssued = XMoveAbsoluteStartCounts(xCountsFromDmm(hole3Rev));
		}
		if (moveIssued && pollMove(Xaxis)) {
			drillRev = DrillRev::Peck3a;
			moveIssued = false;
		}
		return false;
	case DrillRev::Peck3a:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart2));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::Peck3b;
			moveIssued = false;
			Zaxis.VelMax(DRILL_Z_VELOCITY);
		}
		return false;
	case DrillRev::Peck3b:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstop2));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::Peck3c;
			moveIssued = false;
			Zaxis.VelMax(zVelLim);
		}
		return false;
	case DrillRev::Peck3c:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart2));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::XHole4;
			moveIssued = false;
		}
		return false;
	case DrillRev::XHole4:
		if (!moveIssued) {
			moveIssued = XMoveAbsoluteStartCounts(xCountsFromDmm(hole4Rev));
		}
		if (moveIssued && pollMove(Xaxis)) {
			drillRev = DrillRev::Peck4a;
			moveIssued = false;
		}
		return false;
	case DrillRev::Peck4a:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart2));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::Peck4b;
			moveIssued = false;
			Zaxis.VelMax(DRILL_Z_VELOCITY);
		}
		return false;
	case DrillRev::Peck4b:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstop2));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::Peck4c;
			moveIssued = false;
			Zaxis.VelMax(zVelLim);
		}
		return false;
	case DrillRev::Peck4c:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(Zstart2));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::ZEnd;
			moveIssued = false;
		}
		return false;
	case DrillRev::ZEnd:
		if (!moveIssued) {
			moveIssued = ZMoveAbsoluteStartCounts(zCountsFromDmm(180));
		}
		if (moveIssued && pollMove(Zaxis)) {
			drillRev = DrillRev::StopSpindleWait;
			moveIssued = false;
			spindleWaitUntil = 0;
		}
		return false;
	case DrillRev::StopSpindleWait:
		if (spindleWaitUntil == 0) {
			stopSpindle();
			spindleWaitUntil = millis() + 150;
		}
		if ((int32_t)(millis() - spindleWaitUntil) >= 0) {
			drillRev = DrillRev::Done;
		}
		return false;
	case DrillRev::Done:
		return true;
	}
	return true;
}

static bool precisionHomeTick() {
	switch (ph23) {
	case Ph23::RetreatIfBeam2High:
		if (beam2()) {
			Xaxis.MoveVelocity(-1000 * 8);
			motionState = MotionState::MOVING;
		} else {
			Xaxis.MoveStopAbrupt();
			ph23 = Ph23::ForwardFastUntilBeam2Break;
		}
		return false;
	case Ph23::ForwardFastUntilBeam2Break:
		if (!beam2()) {
			Xaxis.MoveVelocity(LPM_COMMISSION_FEED_VELOCITY);
			motionState = MotionState::MOVING;
		} else {
			Xaxis.MoveStopAbrupt();
			ph23 = Ph23::StopAfterFast;
			waitUntilMs = millis() + 50;
		}
		return false;
	case Ph23::StopAfterFast:
		if ((int32_t)(millis() - waitUntilMs) >= 0) {
			ph23 = Ph23::DelayAfterStop1;
			waitUntilMs = millis() + 200;
		}
		return false;
	case Ph23::DelayAfterStop1:
		if ((int32_t)(millis() - waitUntilMs) >= 0) {
			Xaxis.AccelMax(1000 * 8);
			ph23 = Ph23::NudgeBack;
			moveIssued = false;
		}
		return false;
	case Ph23::NudgeBack:
		if (!moveIssued) {
			moveIssued = XMoveDistanceStartCounts(xCountsFromDmm(-50));
		}
		if (moveIssued && pollMove(Xaxis)) {
			ph23 = Ph23::DelayAfterNudge;
			waitUntilMs = millis() + 200;
			moveIssued = false;
		}
		return false;
	case Ph23::DelayAfterNudge:
		if ((int32_t)(millis() - waitUntilMs) >= 0) {
			ph23 = Ph23::CreepAlignBeam2;
		}
		return false;
	case Ph23::CreepAlignBeam2:
		if (!beam2()) {
			Xaxis.MoveVelocity(20);
			motionState = MotionState::MOVING;
		} else {
			Xaxis.MoveStopAbrupt();
			ph23 = Ph23::StopAfterCreep;
			waitUntilMs = millis() + 200;
		}
		return false;
	case Ph23::StopAfterCreep:
		if ((int32_t)(millis() - waitUntilMs) >= 0) {
			ph23 = Ph23::DelayBeforeBeam3;
			waitUntilMs = millis() + 1000;
			xBeforeBeam3Jog = Xaxis.PositionRefCommanded();
			beam3Initial = beam3();
		}
		return false;
	case Ph23::DelayBeforeBeam3:
		if ((int32_t)(millis() - waitUntilMs) >= 0) {
			ph23 = Ph23::Beam3EdgeSeek;
		}
		return false;
	case Ph23::Beam3EdgeSeek:
		if (beam3() == beam3Initial) {
			Xaxis.MoveVelocity(80);
			motionState = MotionState::MOVING;
		} else {
			Xaxis.MoveStopAbrupt();
			xAtBeam3Event = Xaxis.PositionRefCommanded();
			ph23 = Ph23::CheckBeam23Spacing;
		}
		return false;
	case Ph23::CheckBeam23Spacing: {
		int32_t d = xAtBeam3Event - xBeforeBeam3Jog;
		if (d < 0) {
			d = -d;
		}
		int32_t err = d - LPM_BEAM2_TO_BEAM3_EXPECT_COUNTS;
		if (err < 0) {
			err = -err;
		}
		if (err > LPM_BEAM23_SPACING_TOLERANCE_COUNTS) {
			Serial.print("Ph23: beam2-3 spacing check failed, delta=");
			Serial.println((long)d);
		}
		ph23 = Ph23::SetSashRef;
		waitUntilMs = millis() + 1000;
	}
		return false;
	case Ph23::SetSashRef:
		if ((int32_t)(millis() - waitUntilMs) >= 0) {
			Xaxis.PositionRefSet(-1 * (int32_t)SASH_OFFSET);
			Xaxis.AccelMax(xAccelLim);
			ph23 = Ph23::Done;
			motionState = MotionState::IDLE;
		}
		return false;
	case Ph23::Done:
		return true;
	}
	return true;
}

static void resetPrecisionHome() {
	ph23 = Ph23::RetreatIfBeam2High;
	moveIssued = false;
}

static void resetPrecisionRevHome() {
	phRev = PhRev::ForwardFastWhileBeam2High;
	moveIssued = false;
}

static bool precisionRevHomeTick() {
	switch (phRev) {
	case PhRev::ForwardFastWhileBeam2High:
		if (!beam2()) {
			Xaxis.MoveStopAbrupt();
			phRev = PhRev::DelayAfterFast;
			waitUntilMs = millis() - 1;
		} else {
			Xaxis.MoveVelocity(1000 * 8);
			motionState = MotionState::MOVING;
		}
		return false;
	case PhRev::DelayAfterFast:
		if ((int32_t)(millis() - waitUntilMs) >= 0) {
			Xaxis.AccelMax(1000 * 8);
			phRev = PhRev::NudgeBack;
			moveIssued = false;
		}
		return false;
	case PhRev::NudgeBack:
		if (!moveIssued) {
			moveIssued = XMoveDistanceStartCounts(xCountsFromDmm(-50));
		}
		if (moveIssued && pollMove(Xaxis)) {
			phRev = PhRev::DelayAfterNudge;
			waitUntilMs = millis() + 200;
			moveIssued = false;
		}
		return false;
	case PhRev::DelayAfterNudge:
		if ((int32_t)(millis() - waitUntilMs) >= 0) {
			phRev = PhRev::CreepWhileBeam2High;
		}
		return false;
	case PhRev::CreepWhileBeam2High:
		if (beam2()) {
			Xaxis.MoveVelocity(20);
			motionState = MotionState::MOVING;
		} else {
			Xaxis.MoveStopAbrupt();
			phRev = PhRev::DelayFinal;
			waitUntilMs = millis() + 1000;
		}
		return false;
	case PhRev::DelayFinal:
		if ((int32_t)(millis() - waitUntilMs) >= 0) {
			Xaxis.PositionRefSet(-1 * (int32_t)REV_SASH_OFFSET);
			Xaxis.AccelMax(xAccelLim);
			phRev = PhRev::Done;
			motionState = MotionState::IDLE;
		}
		return false;
	case PhRev::Done:
		return true;
	}
	return true;
}

void machineInit() {
	phase = MachinePhase::CommissionFeedX;
	beam1EventArmed = false;
	debounceStableMs = 0;
	sawBeam2Clear = false;
	beam2LowSinceMs = 0;
	drillFwd = DrillFwd::Idle;
	drillRev = DrillRev::Idle;
	resetPrecisionHome();
	resetPrecisionRevHome();
	motionState = MotionState::IDLE;
}

void supervisorTick() {
	digitalWrite(redMastPin, !digitalRead(EstopPin));

	static bool estopStopIssued = false;
	static bool estopWasActive = false;

	if (!digitalRead(EstopPin)) {
		motionState = MotionState::PAUSED_ESTOP;
		digitalWrite(greenMastPin, LOW);
		if (!estopStopIssued) {
			stopAllAxesSmooth();
			estopStopIssued = true;
		}
		estopWasActive = true;
		return;
	}
	estopStopIssued = false;

	// Teknic EStopConnector: after release, ClearAlerts() clears MotionCanceledSensorEStop;
	// then wait for each axis HLFB (drives may be moving to last commanded position).
	if (estopWasActive) {
		Xaxis.ClearAlerts();
		Yaxis.ClearAlerts();
		Zaxis.ClearAlerts();
		estopWasActive = false;
		motionState = MotionState::RECOVERING_ESTOP;
		digitalWrite(greenMastPin, LOW);
	}

	if (motorFaultPresent()) {
		motionState = MotionState::PAUSED_MOTOR_FAULT;
		digitalWrite(greenMastPin, LOW);
		return;
	}
	if (motionState == MotionState::PAUSED_MOTOR_FAULT) {
		motionState = MotionState::IDLE;
		digitalWrite(greenMastPin, HIGH);
	}

	if (motionState == MotionState::RECOVERING_ESTOP) {
		if (allAxesHlfbAsserted()) {
			motionState = MotionState::IDLE;
			digitalWrite(greenMastPin, HIGH);
		} else {
			digitalWrite(greenMastPin, LOW);
			return;
		}
	}

	if (manifoldPressure < g_pressureMinPsi) {
		if (!bypassPressureForBeam1Commissioning()) {
			if (motionState != MotionState::PAUSED_PRESSURE) {
				stopAllAxesSmooth();
			}
			motionState = MotionState::PAUSED_PRESSURE;
			digitalWrite(greenMastPin, LOW);
			return;
		}
	}

	if (motionState == MotionState::PAUSED_PRESSURE) {
		motionState = MotionState::IDLE;
		digitalWrite(greenMastPin, HIGH);
	}
}

void clampManager() {
	if (!motionAllowed()) {
		return;
	}
	int32_t xCmd = Xaxis.PositionRefCommanded();

	struct Band {
		uint8_t clampIndex;
		int32_t engagePast;
		int32_t releasePast;
	};
	static const Band kBands[] = {
	    {2, 1000, 12000},
	    {3, 2500, 14000},
	    {4, 4000, 16000},
	};
	bool engaged[CLAMP_COUNT] = {};

	for (size_t i = 0; i < CLAMP_COUNT; i++) {
		engaged[i] = false;
	}
	for (unsigned i = 0; i < sizeof(kBands) / sizeof(kBands[0]); i++) {
		uint8_t idx = kBands[i].clampIndex;
		if (idx < 1 || idx > CLAMP_COUNT) {
			continue;
		}
		if (xCmd >= kBands[i].engagePast && xCmd < kBands[i].releasePast) {
			engaged[idx - 1] = true;
		}
	}
	for (size_t i = 0; i < CLAMP_COUNT; i++) {
		clamp((int)i + 1, engaged[i]);
	}
}

void machineTick() {
	if (!motionAllowed()) {
		return;
	}

	switch (phase) {
	case MachinePhase::CommissionFeedX:
		if (!beam1EventArmed) {
			beam1Initial = beam1();
			int32_t feedVel = (int32_t)LPM_COMMISSION_FEED_VELOCITY * (int32_t)LPM_COMMISSION_FEED_SIGN;
			Xaxis.MoveVelocity(feedVel);
			motionState = MotionState::MOVING;
			beam1EventArmed = true;
			debounceStableMs = 0;
		} else if (debouncedBeamChange(beam1(), beam1Initial, 5)) {
			Xaxis.MoveStopAbrupt();
			motionState = MotionState::IDLE;
			phase = MachinePhase::RoughSetRef;
			debounceStableMs = 0;
		}
		break;

	case MachinePhase::RoughSetRef:
		Xaxis.PositionRefSet(gBeam1RoughRefCounts);
		sawBeam2Clear = false;
		beam2LowSinceMs = 0;
		phase = MachinePhase::ClampingRun;
		break;

	case MachinePhase::ClampingRun:
		clampManager();
		if (beam2()) {
			sawBeam2Clear = true;
			beam2LowSinceMs = 0;
		} else if (sawBeam2Clear) {
			if (beam2LowSinceMs == 0) {
				beam2LowSinceMs = millis();
			}
			if (millis() - beam2LowSinceMs > 20) {
				resetPrecisionHome();
				phase = MachinePhase::PrecisionHome;
			}
		}
		break;

	case MachinePhase::PrecisionHome:
		motionState = MotionState::MOVING;
		if (precisionHomeTick()) {
#if LPM_ENABLE_DRILL_CYCLE
			resetDrillFwd();
			phase = MachinePhase::DrillForward;
			CcioMgr.PinByIndex(yellowMastPin)->OutputPulsesStart(1000, 400, 0, false);
#else
			phase = MachinePhase::CycleDoneWait;
			waitUntilMs = millis() + 500;
#endif
		}
		break;

	case MachinePhase::DrillForward:
		motionState = MotionState::MOVING;
		if (drillFwdTick()) {
			phase = MachinePhase::AfterDrillMoveX;
			moveIssued = false;
		}
		break;

	case MachinePhase::AfterDrillMoveX:
		motionState = MotionState::MOVING;
		if (!moveIssued) {
			moveIssued = XMoveAbsoluteStartCounts(xCountsFromDmm(1000));
		}
		if (moveIssued && pollMove(Xaxis)) {
			resetPrecisionRevHome();
			phase = MachinePhase::PrecisionRevHome;
			moveIssued = false;
		}
		break;

	case MachinePhase::PrecisionRevHome:
		motionState = MotionState::MOVING;
		if (precisionRevHomeTick()) {
			resetDrillRev();
			phase = MachinePhase::DrillReverse;
		}
		break;

	case MachinePhase::DrillReverse:
		motionState = MotionState::MOVING;
		if (drillRevTick()) {
			phase = MachinePhase::EjectSeek;
		}
		break;

	case MachinePhase::EjectSeek:
		motionState = MotionState::MOVING;
		if (beam2()) {
			Xaxis.MoveVelocity(4000 * 8);
		} else {
			Xaxis.MoveStopAbrupt();
			Xaxis.VelMax(2000 * 8);
			phase = MachinePhase::EjectDistance;
			moveIssued = false;
		}
		break;

	case MachinePhase::EjectDistance:
		motionState = MotionState::MOVING;
		if (!moveIssued) {
			// Legacy sketch used XMoveDistance(1380*8) (DMM field overloaded); preserve distance.
			int32_t ejectCounts = (int32_t)lround(1380.0 * 8.0 * X_STEPS_PER_DMM);
			moveIssued = XMoveDistanceStartCounts(ejectCounts);
		}
		if (moveIssued && pollMove(Xaxis)) {
			Xaxis.VelMax(xVelLim);
			CcioMgr.PinByIndex(yellowMastPin)->OutputPulsesStop();
			CcioMgr.PinByIndex(buzzMastPin)->OutputPulsesStart(250, 250, 1, false);
			phase = MachinePhase::CycleDoneWait;
			waitUntilMs = millis() + 1000;
		}
		break;

	case MachinePhase::CycleDoneWait:
		if ((int32_t)(millis() - waitUntilMs) >= 0) {
			machineInit();
		}
		break;
	}
}
