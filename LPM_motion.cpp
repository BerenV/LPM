#include "LPM_motion.h"

bool motorFaultPresent() {
	return Xaxis.StatusReg().bit.AlertsPresent || Yaxis.StatusReg().bit.AlertsPresent
	       || Zaxis.StatusReg().bit.AlertsPresent;
}
