/*
 * Controller_MidLevel_SpeedLoop.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: jespestana
 */

#include "controller/midlevelCnt/Controller_MidLevel_SpeedLoop.h"

Controller_MidLevel_SpeedLoop::Controller_MidLevel_SpeedLoop() :  started(false) {

	// Just in case, I set to zero all the internal variables of the controller object
	vxci = 0.0; vyci = 0.0; yawci = 0.0; zci = 0.0;
	vxs = 0.0; vys = 0.0; yaws = 0.0; zs = 0.0;
	pitchd = 0.0; rolld = 0.0; eps_yaw = 0.0; eps_z = 0.0; vxd_int = 0.0; vyd_int = 0.0;
	pitchc_int = 0.0; rollc_int = 0.0; dyawc_int = 0.0; dzc_int = 0.0;
	pitchco = 0.0; rollco = 0.0; dyawco = 0.0; dzco = 0.0;

	// Nacho's PID algorithm >>>> u[k] = Kp*e[k] + Ki*i[k] + Kd*de[k]

	// Vx PID gains: Kp, Ki = Kp/Ti, Kd = Kp*Td
	pid_vx.setGains( 	MULTIROTOR_SPEEDCONTROLLER_VX_KP,
						MULTIROTOR_SPEEDCONTROLLER_VX_KI,
						MULTIROTOR_SPEEDCONTROLLER_VX_KD); // set Kp, Ki = Kp/Ti, Kd = Kp*Td
#ifdef CONTROLLER_DEACTIVATE_NLF
	// Saturation specified by config file
	pid_vx.enableAntiWindup(true, MULTIROTOR_SPEEDCONTROLLER_MAX_PITCH);
	pid_vx.enableMaxOutput( true, MULTIROTOR_SPEEDCONTROLLER_MAX_PITCH); // set enable, value. Maximum output is vx_max = 2.5-2.75 m/s
#else
	//  Saturation performed by non-linear function
	 // (Parrot 1) Maximum output is vx_max = 2.5-2.75 m/s
	pid_vx.enableAntiWindup(true, MULTIROTOR_SPEEDCONTROLLER_VMAX);
	pid_vx.enableMaxOutput( true, MULTIROTOR_SPEEDCONTROLLER_VMAX);
#endif

	// Vy PID gains: Kp, Ki = Kp/Ti, Kd = Kp*Td
	pid_vy.setGains( 	MULTIROTOR_SPEEDCONTROLLER_VY_KP,
						MULTIROTOR_SPEEDCONTROLLER_VY_KI,
						MULTIROTOR_SPEEDCONTROLLER_VY_KD);
#ifdef CONTROLLER_DEACTIVATE_NLF
	// Saturation specified by config file
	pid_vy.enableAntiWindup(true, MULTIROTOR_SPEEDCONTROLLER_MAX_ROLL);
	pid_vy.enableMaxOutput( true, MULTIROTOR_SPEEDCONTROLLER_MAX_ROLL);
#else
	//  Saturation performed by non-linear function
	// (Parrot 1) Maximum output is vy_max = 2.0-2.30 m/s
	pid_vy.enableAntiWindup(true, MULTIROTOR_SPEEDCONTROLLER_VMAX);
	pid_vy.enableMaxOutput( true, MULTIROTOR_SPEEDCONTROLLER_VMAX);
#endif

	// yaw PID gains: Kp, Ki = Kp/Ti, Kd = Kp*Td
	pid_yaw.setGains( 	MULTIROTOR_SPEEDCONTROLLER_YAW_KP,
						MULTIROTOR_SPEEDCONTROLLER_YAW_KI,
						MULTIROTOR_SPEEDCONTROLLER_YAW_KD);
	pid_yaw.enableMaxOutput(true,  MULTIROTOR_SPEEDCONTROLLER_DYAWMAX);
	pid_yaw.enableMaxOutput( true, MULTIROTOR_SPEEDCONTROLLER_DYAWMAX);

	// z PID gains: Kp, Ki = Kp/Ti, Kd = Kp*Td
	pid_z.setGains( 	MULTIROTOR_SPEEDCONTROLLER_Z_KP,
						MULTIROTOR_SPEEDCONTROLLER_Z_KI,
						MULTIROTOR_SPEEDCONTROLLER_Z_KD);
	pid_z.enableMaxOutput( true,  MULTIROTOR_SPEEDCONTROLLER_DZMAX);
	pid_z.enableMaxOutput( true,  MULTIROTOR_SPEEDCONTROLLER_DZMAX);

	// Important: Default control mode is set in Controller_Multirotor::Controller_Multirotor() code.
	control_mode = Controller_MidLevel_controlMode::POSITION_CONTROL;
	setControlMode(control_mode);
}

Controller_MidLevel_SpeedLoop::~Controller_MidLevel_SpeedLoop() {
}

void Controller_MidLevel_SpeedLoop::getOutput( cvg_double *pitchco_out, cvg_double *rollco_out, cvg_double *dyawco_out, cvg_double *dzco_out) {


	if (!started) {
		reset();
		*pitchco_out = 0.0;
		*rollco_out  = 0.0;
		*dyawco_out  = 0.0;
		*dzco_out    = 0.0;
		started = true;
		return;
	}

	// Saturate speed reference input to MULTIROTOR_SPEEDCONTROLLER_VMAX
	cvg_double vrci = sqrt( vxci*vxci + vyci*vyci);
	if ( vrci  > MULTIROTOR_SPEEDCONTROLLER_VXY_MAX ) {
		cvg_double Kaux = MULTIROTOR_SPEEDCONTROLLER_VXY_MAX / vrci;
		vxci = Kaux * vxci;
		vyci = Kaux * vyci;
	}

	// Perform PID_VX calculations and obtain vxd_int
	pid_vx.setReference(vxci);
	pid_vx.setFeedback(vxs);
	vxd_int = pid_vx.getOutput();
	// Perform PID_VY calculations and obtain vyd_int
	pid_vy.setReference(vyci);
	pid_vy.setFeedback(vys); // I put 0.0 because eps_vym is already (Reference - Feedback)
	vyd_int = pid_vy.getOutput();

	// Compute {Pitchd, Rolld} desired values from {vxd, vyd}
	referenceChangeFixed2Moving( vxd_int, vyd_int, yaws, &pitchd, &rolld);
	// Compute {Pitch} command from {Pitchd}
#ifdef CONTROLLER_DEACTIVATE_NLF
	pitchc_int = (-1)*pitchd;
#else
	nlf_p2vx( pitchd, &pitchc_int);
#endif
//		pitchco_saturation( pitchc_int, &pitchco); // Commented because: Saturation already performed either by NLF or by PID
	pitchco = pitchc_int;

	// Compute {Roll} command from {Rolld}
#ifdef CONTROLLER_DEACTIVATE_NLF
	rollc_int = rolld;
#else
	nlf_r2vy( rolld, &rollc_int);
#endif
	//	rollco_saturation( rollc_int, &rollco); // Commented because: Saturation already performed either by NLF or by PID
	rollco = rollc_int;

	// Compute {dYawdt} output command from {Yawc} input command
	pid_yaw.setReference(yawci);
	pid_yaw.setFeedback(yaws);
	dyawc_int = pid_yaw.getOutput();
	//	dyawco_saturation( dyawc_int, &dyawco); // Commented because: Saturation already performed either by PID
	dyawco = dyawc_int;

	// Compute {Z} output command from {Z} input command
	pid_z.setReference(zci);
	pid_z.setFeedback(zs);
	dzc_int = pid_z.getOutput();
	//	dzco_saturation( dzc_int, &dzco); // Commented because: Saturation already performed either by PID
	dzco = dzc_int;

	*pitchco_out = pitchco;
	*rollco_out  = rollco;
	*dyawco_out  = dyawco;
	*dzco_out    = dzco;

}

void Controller_MidLevel_SpeedLoop::reset() {

		// Just in case, I set to zero all the variables of the controller object
//		setYawAndZ2ActualValues(); // yawci = 0.0; zci = 0.0;
		vxci = 0.0; vyci = 0.0; vxs = 0.0; vys = 0.0; yaws = 0.0; zs = 0.0;
		pitchd = 0.0; rolld = 0.0; eps_yaw = 0.0; eps_z = 0.0; vxd_int = 0.0; vyd_int = 0.0;
		pitchc_int = 0.0; rollc_int = 0.0; dyawc_int = 0.0; dzc_int = 0.0;
		pitchco = 0.0; rollco = 0.0; dyawco = 0.0; dzco = 0.0;

		started = false;

		pid_vx.reset();
		pid_vy.reset();
		pid_z.reset();
		pid_yaw.reset();

}

void Controller_MidLevel_SpeedLoop::referenceChangeFixed2Moving(cvg_double eps_vxs_f, cvg_double eps_vys_f, cvg_double yaws, cvg_double *eps_vxm, cvg_double *eps_vym) {

	*eps_vxm =  cos(yaws)*eps_vxs_f + sin(yaws)*eps_vys_f;
	*eps_vym = -sin(yaws)*eps_vxs_f + cos(yaws)*eps_vys_f;

}

void Controller_MidLevel_SpeedLoop::nlf_p2vx(cvg_double vxmd_int, cvg_double *pitchc_int) {

	cvg_double vx_max = 2.625; // m/s

	if ( fabs(vxmd_int) > vx_max ) {
		vxmd_int = vx_max* ( (vxmd_int > 0) ? 1 : ((vxmd_int < 0) ? -1 : 0) );
	}

	*pitchc_int = -(1.0/2.703)*asin(vxmd_int/vx_max);
//	*pitchc_int = (cvg_double) ( -(1.0/2.703)*asin(vxmd_int/vx_max) );

}

void Controller_MidLevel_SpeedLoop::pitchco_saturation(cvg_double pitchc_int, cvg_double *pitchco) {

	// double jesus_library::saturate(double x, double x_lim_inf, double x_lim_sup);
	*pitchco = jesus_library::saturate(pitchc_int, -1.0, 1.0);

}

void Controller_MidLevel_SpeedLoop::nlf_r2vy(cvg_double vymd_int, cvg_double *rollc_int) {

	cvg_double vy_max = 2.172; // m/s

	if ( fabs(vymd_int) > vy_max ) {
		vymd_int = vy_max* ( (vymd_int > 0) ? 1 : ((vymd_int < 0) ? -1 : 0) );
	}

	*rollc_int = (1/2.961)*asin(vymd_int/vy_max);
//	*rollc_int = (cvg_double) ( -(1.0/2.703)*asin(vymd_int/vy_max) );

}

void Controller_MidLevel_SpeedLoop::rollco_saturation(cvg_double rollc_int, cvg_double *rollco) {

	// double jesus_library::saturate(double x, double x_lim_inf, double x_lim_sup);
	*rollco = jesus_library::saturate(rollc_int, -1.0, 1.0);

}

void Controller_MidLevel_SpeedLoop::dyawco_saturation(cvg_double dyawc_int, cvg_double *dyawco) {

	// double jesus_library::saturate(double x, double x_lim_inf, double x_lim_sup);
	*dyawco = jesus_library::saturate(dyawc_int, -1.0, 1.0);

}

void Controller_MidLevel_SpeedLoop::dzco_saturation(cvg_double dzc_int, cvg_double *dzco) {

	// double jesus_library::saturate(double x, double x_lim_inf, double x_lim_sup);
//	*dzco = jesus_library::saturate(dzc_int, -1.0, 1.0);
	*dzco = jesus_library::saturate(dzc_int, -0.4, 0.4);

}

void Controller_MidLevel_SpeedLoop::setControlMode(Controller_MidLevel_controlMode::controlMode mode){

	switch (mode) {
	case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
		pid_z.enableMaxOutput( true,  MULTIROTOR_TRAJECTORYCONTROLLER_VZ_CT_MAX);
		pid_z.enableAntiWindup( true,  MULTIROTOR_TRAJECTORYCONTROLLER_VZ_CT_MAX);
		break;
	case Controller_MidLevel_controlMode::POSITION_CONTROL:
	case Controller_MidLevel_controlMode::SPEED_CONTROL:
		pid_z.enableMaxOutput( true,  MULTIROTOR_SPEEDCONTROLLER_DZMAX);
		pid_z.enableAntiWindup( true,  MULTIROTOR_SPEEDCONTROLLER_DZMAX);
		break;
	}

	control_mode = mode;
}

Controller_MidLevel_controlMode::controlMode Controller_MidLevel_SpeedLoop::getControlMode() {
	return control_mode;
}
