#include "LPM_motion.h"

bool motorFaultPresent() {
	return axisMotionFault(Xaxis) || axisMotionFault(Yaxis) || axisMotionFault(Zaxis);
}
