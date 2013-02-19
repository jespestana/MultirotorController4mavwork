/*
 * Controller_MidLevelCnt.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: jespestana
 */

#include "controller/midlevelCnt/Controller_MidLevelCnt.h"

Controller_MidLevelCnt::Controller_MidLevelCnt() : started(false) {

	// Just in case, I set to zero all the variables of the controller object
	xci = 0; yci = 0; yawci = 0; zci = 0;
	vxfi = 0.0; vyfi = 0.0; dyawfi = 0.0; dzfi = 0.0; pitchfi = 0.0; rollfi = 0.0;
	xs = 0; ys = 0; vxs = 0; vys = 0; yaws = 0; zs = 0;
	eps_x = 0; eps_y = 0;
	vxc_int = 0; vyc_int = 0;
	vxco_int = 0; vyco_int = 0; yawco_int = 0; zco_int = 0;
	pitchco = 0; rollco = 0; dyawco = 0; dzco = 0;

	// Reset speed controller
	speedController.reset();

	pid_x.setGains( 	MULTIROTOR_POSITIONCONTROLLER_VXVY_KP,
						MULTIROTOR_POSITIONCONTROLLER_VXVY_KI,
						MULTIROTOR_POSITIONCONTROLLER_VXVY_KD);
	pid_x.enableMaxOutput(true,  MULTIROTOR_SPEEDCONTROLLER_VXY_MAX);
	pid_x.enableAntiWindup(true, MULTIROTOR_SPEEDCONTROLLER_VXY_MAX);

	pid_y.setGains( 	MULTIROTOR_POSITIONCONTROLLER_VXVY_KP,
						MULTIROTOR_POSITIONCONTROLLER_VXVY_KI,
						MULTIROTOR_POSITIONCONTROLLER_VXVY_KD);
	pid_y.enableMaxOutput( true, MULTIROTOR_SPEEDCONTROLLER_VXY_MAX);
	pid_y.enableAntiWindup(true, MULTIROTOR_SPEEDCONTROLLER_VXY_MAX);

	// Important: Default control mode is set in Controller_Multirotor::Controller_Multirotor() code.
	control_mode = Controller_MidLevel_controlMode::POSITION_CONTROL;
	setControlMode(control_mode);

}

Controller_MidLevelCnt::~Controller_MidLevelCnt() {
}

void Controller_MidLevelCnt::setFeedback( cvg_double xs_t, cvg_double ys_t, cvg_double vxs_t, cvg_double vys_t, cvg_double yaws_t, cvg_double zs_t) {
	xs = xs_t; ys = ys_t;
	vxs = vxs_t; vys = vys_t;
	yaws = yaws_t;
	zs = zs_t;
}

void Controller_MidLevelCnt::setReference( cvg_double xci_t, cvg_double yci_t, cvg_double yawci_t, cvg_double zci_t,
		cvg_double vxfi_t , cvg_double vyfi_t , cvg_double dyawfi_t , cvg_double dzfi_t ,
		cvg_double pitchfi_t , cvg_double rollfi_t ) {
	xci = xci_t; yci = yci_t; yawci = yawci_t; zci = zci_t;
	vxfi = vxfi_t; vyfi = vyfi_t; dyawfi = dyawfi_t; dzfi = dzfi_t;
	pitchfi = pitchfi_t; rollfi = rollfi_t;
}

void Controller_MidLevelCnt::getOutput( cvg_double *pitchco_out, cvg_double *rollco_out, cvg_double *dyawco_out, cvg_double *dzco_out) {

	if (!started) {
		reset();
		*pitchco_out = 0.0;
		*rollco_out  = 0.0;
		*dyawco_out  = 0.0;
		*dzco_out    = 0.0;
		started = true;
		return;
	}

	// Calculate vxc for speed controller
	pid_x.setReference(xci);
	pid_x.setFeedback(xs);
	vxc_int = pid_x.getOutput();

	// Calculate vyc for speed controller
	pid_y.setReference(yci);
	pid_y.setFeedback(ys);
	vyc_int = pid_y.getOutput();

	switch (control_mode) {
	case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
		saturation_2D( vxc_int, vyc_int, &vxc_int, &vyc_int, MULTIROTOR_TRAJECTORYCONTROLLER_VXY_CT_MAX);
		saturation_2D( vxfi, vyfi, &vxfi, &vyfi, MULTIROTOR_TRAJECTORYCONTROLLER_VXY_AT_MAX);
		vxco_int = vxc_int + vxfi;
		vyco_int = vyc_int + vyfi;
		dzfi = jesus_library::saturate(dzfi, -MULTIROTOR_TRAJECTORYCONTROLLER_VZ_AT_MAX, MULTIROTOR_TRAJECTORYCONTROLLER_VZ_AT_MAX);
		// No yaw AT, CT saturation considered for now; dyawfi will usually be 0.0
		break;
	case Controller_MidLevel_controlMode::POSITION_CONTROL:
		vxfi = 0.0;
		vyfi = 0.0;
		dzfi = 0.0;
		// No yaw AT, CT saturation considered for now; dyawfi will usually be 0.0
		// No saturation command here, why?: saturation of speed reference on speed controller
		vxco_int = vxc_int;
		vyco_int = vyc_int;
		break;
	case Controller_MidLevel_controlMode::SPEED_CONTROL:
		// No saturation command here, why?: saturation of speed reference on speed controller
		vxco_int = vxfi;
		vyco_int = vyfi;
		dzfi = 0.0;	// altitude is also position controlled in this mode
		// No yaw AT, CT saturation considered for now; dyawfi will usually be 0.0
		break;
	}


	// Yaw and Z commands pass directly to speed controller
	yawco_int = yawci;
	zco_int = zci;

	// Enter references to speed controller
	speedController.setReference( vxco_int, vyco_int, yawci, zci);
	// Enter feedback measurements/estimations to speed controller
	speedController.setFeedback( vxs, vys, yaws, zs);
	// Obtain controller outputs (control commands for the pelican proxy)
	speedController.getOutput( &pitchco, &rollco, &dyawco, &dzco);

	// Commented because: Saturation already performed inside speed controller (avoid code duplicates if possible)
//	pitchco = jesus_library::saturate(  pitchco, 		-MULTIROTOR_SPEEDCONTROLLER_MAX_PITCH, +MULTIROTOR_SPEEDCONTROLLER_MAX_PITCH);
//	rollco  = jesus_library::saturate(    rollco,  		-MULTIROTOR_SPEEDCONTROLLER_MAX_ROLL,  +MULTIROTOR_SPEEDCONTROLLER_MAX_ROLL);
//	dyawco  = jesus_library::saturate(    dyawco,   	-MULTIROTOR_SPEEDCONTROLLER_DYAWMAX,   +MULTIROTOR_SPEEDCONTROLLER_DYAWMAX);
//	dzco    = jesus_library::saturate(        dzco,     -MULTIROTOR_SPEEDCONTROLLER_DZMAX,     +MULTIROTOR_SPEEDCONTROLLER_DZMAX);

	*pitchco_out = pitchco + pitchfi;
	*rollco_out  = rollco  + rollfi;
	*dyawco_out  = dyawco  + dyawfi;
	*dzco_out    = dzco    + dzfi;

}

void Controller_MidLevelCnt::reset() {

	// Just in case, I set to zero all the variables of the controller object
	xci = 0; yci = 0; yawci = 0; zci = 0;
	vxfi = 0.0; vyfi = 0.0; dyawfi = 0.0; dzfi = 0.0; pitchfi = 0.0; rollfi = 0.0;
	xs = 0; ys = 0; vxs = 0; vys = 0; yaws = 0; zs = 0;
	eps_x = 0; eps_y = 0;
	vxc_int = 0; vyc_int = 0;
	vxco_int = 0; vyco_int = 0; yawco_int = 0; zco_int = 0;
	pitchco = 0; rollco = 0; dyawco = 0; dzco = 0;

	started = false;
	// Reset speed controller
	speedController.reset();

	// Reset internal PIDs
	pid_x.reset();
	pid_y.reset();
}

void Controller_MidLevelCnt::getIntermediateVars( cvg_double *vxco_int_out, cvg_double *vyco_int_out, cvg_double *yawco_int_out, cvg_double *zco_int_out) {
	*vxco_int_out  = vxco_int;
	*vyco_int_out  = vyco_int;
	*yawco_int_out = yawco_int;
	*zco_int_out   = zco_int;
}

void Controller_MidLevelCnt::saturation_2D(cvg_double x1, cvg_double x2, cvg_double *y1, cvg_double *y2, cvg_double max) {

	cvg_double modulus = sqrt( pow( x1 ,2) + pow( x2 ,2) );

	if ( fabs(modulus) > max ) {
		*y1 = (max/modulus)*x1;
		*y2 = (max/modulus)*x2;
	} else {
		*y1 = x1;
		*y2 = x2;
	}

}

void Controller_MidLevelCnt::setControlMode(Controller_MidLevel_controlMode::controlMode mode) {

	switch (mode) {
	case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
		pid_x.enableMaxOutput( true, MULTIROTOR_TRAJECTORYCONTROLLER_VXY_CT_MAX);
		pid_x.enableAntiWindup(true, MULTIROTOR_TRAJECTORYCONTROLLER_VXY_CT_MAX);
		pid_y.enableMaxOutput( true, MULTIROTOR_TRAJECTORYCONTROLLER_VXY_CT_MAX);
		pid_y.enableAntiWindup(true, MULTIROTOR_TRAJECTORYCONTROLLER_VXY_CT_MAX);
		break;
	case Controller_MidLevel_controlMode::POSITION_CONTROL:
	case Controller_MidLevel_controlMode::SPEED_CONTROL:
		pid_x.enableMaxOutput(true,  MULTIROTOR_SPEEDCONTROLLER_VXY_MAX);
		pid_x.enableAntiWindup(true, MULTIROTOR_SPEEDCONTROLLER_VXY_MAX);
		pid_y.enableMaxOutput( true, MULTIROTOR_SPEEDCONTROLLER_VXY_MAX);
		pid_y.enableAntiWindup(true, MULTIROTOR_SPEEDCONTROLLER_VXY_MAX);
		break;
	}

	speedController.setControlMode(mode);

	control_mode = mode;
}

Controller_MidLevel_controlMode::controlMode Controller_MidLevelCnt::getControlMode() {
	return control_mode;
}
