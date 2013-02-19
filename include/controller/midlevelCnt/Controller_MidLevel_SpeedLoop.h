/*
 * Controller_MidLevel_SpeedLoop.h
 *
 *  Created on: Nov 20, 2012
 *      Author: jespestana
 */

#ifndef CONTROLLER_MIDLEVEL_SPEEDLOOP_H_
#define CONTROLLER_MIDLEVEL_SPEEDLOOP_H_

#include <atlante.h>
#include "Other/jesus_library.h"

#include <math.h>

#include "controller/other/PID.h"

#include "config_Mydrone.h" // Which drone is going to be used?

#ifdef _MULTIROTOR_IS_PARROT_
#include "controller/config/parrot/config_controller_Parrot.h"
#endif
#ifdef _MULTIROTOR_IS_PELICAN_
#include "controller/config/pelican/config_controller_Pelican.h"
#endif

#include "controller/midlevelCnt/Controller_MidLevel_controlModes.h"

class Controller_MidLevel_SpeedLoop {
private:
	// Controller command inputs
	cvg_double vxci, vyci, yawci, zci;			// ci ~ command inputs
	// Controller state estimate inputs (controller feedback)
	cvg_double vxs, vys, yaws, zs;
	// Intermediate variables between controller blocks
	cvg_double eps_vx, eps_vy, eps_yaw, eps_z;// eps ~ epsilon to denote the control error in the commanded variable
	cvg_double vxd_int, vyd_int;				// d ~ desired value; int ~ internal variable
	cvg_double pitchd, rolld;
	cvg_double pitchc_int, rollc_int, dyawc_int, dzc_int;
	// Next control layer commands, in this case these commands are to be sent directly to the drone using the parrot proxy
	cvg_double pitchco, rollco, dyawco, dzco;	// co ~ command outputs

	// Internal blocks of the controller diagram, PID and others
	CVG_BlockDiagram::PID pid_vx, pid_vy, pid_z, pid_yaw;
	void referenceChangeFixed2Moving(cvg_double eps_vxs_f, cvg_double eps_vys_f, cvg_double yaws, cvg_double *eps_vxm, cvg_double *eps_vym);
	void nlf_p2vx(cvg_double vxmd_int, cvg_double *pitchc_int);
	void pitchco_saturation(cvg_double pitchc_int, cvg_double *pitchco);
	void nlf_r2vy(cvg_double vymd_int, cvg_double *rollc_int);
	void rollco_saturation(cvg_double rollc_int, cvg_double *rollco);
	void dyawco_saturation(cvg_double dyawc_int, cvg_double *dyawco);
	void dzco_saturation(cvg_double dzc_int, cvg_double *dzco);

	volatile cvg_bool started;
	Controller_MidLevel_controlMode::controlMode control_mode;

protected:

public:
	Controller_MidLevel_SpeedLoop();
	~Controller_MidLevel_SpeedLoop();

	void reset();

	inline void setFeedback( cvg_double vxs_t, cvg_double vys_t, cvg_double yaws_t, cvg_double zs_t) {vxs = vxs_t; vys = vys_t; yaws = yaws_t; zs = zs_t;}
	inline void setReference( cvg_double vxci_t, cvg_double vyci_t, cvg_double yawci_t, cvg_double zci_t) {vxci = vxci_t; vyci = vyci_t; yawci = yawci_t; zci = zci_t;}
	void getOutput( cvg_double *pitchco, cvg_double *rollco, cvg_double *dyawco, cvg_double *dzco);

	void setControlMode(Controller_MidLevel_controlMode::controlMode mode);
	Controller_MidLevel_controlMode::controlMode getControlMode();

};

#endif /* CONTROLLER_MIDLEVEL_SPEEDLOOP_H_ */
