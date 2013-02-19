/*
 * Controller_MidLevelCnt.h
 *
 *  Created on: Nov 20, 2012
 *      Author: jespestana
 */

#ifndef CONTROLLER_MIDLEVELCNT_H_
#define CONTROLLER_MIDLEVELCNT_H_

#include "config_Mydrone.h" // Which drone is going to be used?

#ifdef _MULTIROTOR_IS_PARROT_
#include "controller/config/parrot/config_controller_Parrot.h"
#endif
#ifdef _MULTIROTOR_IS_PELICAN_
#include "controller/config/pelican/config_controller_Pelican.h"
#endif

#include "controller/midlevelCnt/Controller_MidLevel_SpeedLoop.h"
#include "controller/midlevelCnt/Controller_MidLevel_controlModes.h"
#include <atlante.h>
#include <math.h>

class Controller_MidLevelCnt {
private:
	// Controller command inputs
	cvg_double xci, yci, yawci, zci;			// ci ~ command inputs
	cvg_double vxfi, vyfi, dyawfi, dzfi, pitchfi, rollfi;		// fi ~ feedforward inputs
	// Controller state estimate inputs (controller feedback)
	cvg_double xs, ys, vxs, vys, yaws, zs;
	// Intermediate variables between controller blocks
	cvg_double eps_x, eps_y;					// eps ~ epsilon to denote the control error in the commanded variable
	cvg_double vxc_int, vyc_int;				// d ~ desired value; int ~ internal variable
	// Next control layer commands, in this case these commands are to be sent directly to the drone using the pelican proxy
	cvg_double vxco_int, vyco_int, yawco_int, zco_int;	// co_int ~ command outputs internal (to speed controller)
	cvg_double pitchco, rollco, dyawco, dzco;			// co ~ command outputs (to pelican proxy)

	// Internal PID controllers
	CVG_BlockDiagram::PID pid_x, pid_y;
	void saturation_2D(cvg_double x1, cvg_double x2, cvg_double *y1, cvg_double *y2, cvg_double max);

	// Internal speed controller
	Controller_MidLevel_SpeedLoop speedController;

	volatile cvg_bool started;
	Controller_MidLevel_controlMode::controlMode control_mode;

public:
	Controller_MidLevelCnt();
	~Controller_MidLevelCnt();

	void reset();

	void setFeedback( cvg_double xs_t, cvg_double ys_t, cvg_double vxs_t, cvg_double vys_t, cvg_double yaws_t, cvg_double zs_t);
	void setReference( cvg_double xci_t, cvg_double yci_t, cvg_double yawci_t, cvg_double zci_t,
			cvg_double vxfi_t = 0.0, cvg_double vyfi_t = 0.0, cvg_double dyawfi_t = 0.0, cvg_double dzfi_t = 0.0,
			cvg_double pitchfi_t = 0.0, cvg_double rollfi_t = 0.0);
	void getOutput( cvg_double *pitchco, cvg_double *rollco, cvg_double *dyawco, cvg_double *dzco);

	void setControlMode(Controller_MidLevel_controlMode::controlMode mode);
	Controller_MidLevel_controlMode::controlMode getControlMode();

	void getIntermediateVars( cvg_double *vxco_int_out, cvg_double *vyco_int_out, cvg_double *yawco_int_out, cvg_double *zco_int_out);
};


#endif /* CONTROLLER_MIDLEVELCNT_H_ */
