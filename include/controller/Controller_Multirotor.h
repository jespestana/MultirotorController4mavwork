/*
 * Controller_Multirotor.h
 *
 *  Created on: Nov 20, 2012
 *      Author: jespestana
 */

#ifndef MULTIROTOR_CONTROLLER_H_
#define MULTIROTOR_CONTROLLER_H_

// My drone related headers
#include "config_Mydrone.h" // Which drone is going to be used?
class MyDrone;

// Proxy and proxy libraries
#include <atlante.h>

// state observer
#include "stateObserver/EKF_Multirotor.h"
typedef EKF_Multirotor StateObserver;

// controller
#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
#include "controller/midlevelCnt/Controller_MidLevel_controlModes.h"
#include "controller/stateMachine/Controller_StateMachine_v2.h"
#include "controller/stateMachine/Controller_SM_stateNames.h"
#include "controller/stateMachine/Controller_SM_Trajectory_v2.h"

// other resources
#include "controller/other/PID.h"
#include "controller/other/LowPassFilter.h"
#include "Other/jesus_library.h"
#include <math.h>

#ifdef _MULTIROTOR_IS_PARROT_
#endif
#ifdef _MULTIROTOR_IS_PELICAN_
#endif

// to-do list:
//	 TODO: Adapt the controller code to be used by the scheduler (the scheduler is right now commented...)
//   REQUIRES_REDEBUGGING_CODE_THAT_WORKS>>THUS_NOT_WORTH_DOING: The X-Y, VX-VY controllers should have the PIDs and the saturations united (something like 2D-saturated-antiwinduped-PID)
// 	 DONE: Current multirotor reference frame is (X>front-facing) (Y>right-facing) (Z>down-facing)
//	 TODO: make explanatory documentation about the controller (where are the saturations, etc)
//	 			- TODO: explain meaning of configuration constants
//				- TODO: check compatibility between the proxy, the model, controller and scheduler about reference frames/systems

class Controller_Multirotor : public virtual cvgThread {

	// ************** Things that DO NOT NEED to be changed for each controller ****************
private:
	//	***** controller thread related data *****
	volatile cvg_bool controller_started;
	Controller_MidLevel_controlMode::controlMode control_mode;
	DroneProxy::Threading::Mutex controllerMutex;
	DroneProxy::Timing::Timer timer;
	cvg_double Ts;
protected:
	void run();
	//	END: ***** controller thread related data *****

private:
	// Access to global timer and EKF and other common resources
	MyDrone *pmy_drone; // pointer to my_drone: timer, EKF, and logging through events
	Vector estObserv;

public:
	Controller_Multirotor();
	void setMyDrone(MyDrone *p2my_drone); // use in MyDrone constructor
	~Controller_Multirotor();

private:
	void reset();
public:
	inline cvg_bool isStarted() { return controller_started; }
	void startController();
	void stopController();
	// END END END  * Things that DO NOT NEED to be changed for each controller **** END END END

	// ************** Things that have to be changed for each controller ****************
private:
	// ***** Internal variables in the controller block diagram *****
	// Controller command inputs
	cvg_double xci, yci, yawci, zci;			// ci ~ command inputs
	cvg_double vxfi, vyfi, dyawfi, dzfi, pitchfi, rollfi;		// fi ~ feedforward inputs
	// Controller state estimate inputs (controller feedback)
	cvg_double xs, ys, vxs, vys, vzs, yaws, dyaws, zs;
	// Next control layer commands, in this case these commands are to be sent directly to the drone using the pelican proxy
	cvg_double vxco_int, vyco_int, yawco_int, zco_int;	// co_int ~ command outputs internal (to speed controller)
	cvg_double pitchco_hf, rollco_hf, dyawco_hf, dzco_hf; // hf ~ "high frequency", before (low pass) filtering
	cvg_double pitchco, rollco, dyawco, dzco;			// co ~ command outputs (to pelican proxy)

	Controller_StateMachine_v2 		state_machine;	// enables switching speed, position and trajectory control
	Controller_MidLevelCnt			midlevel_controller;
	CVG_BlockDiagram::LowPassFilter pitch_lowpassfilter, roll_lowpassfilter, dyaw_lowpassfilter, dz_lowpassfilter;
	// ***** END: Internal variables in the controller block diagram *****

public:

	// ***** Set controller reference functions *****
	void setAllReferences( cvg_double xci_in, cvg_double yci_in, cvg_double yawci_in, cvg_double zci_in,
			cvg_double vxfi_t = 0.0, cvg_double vyfi_t = 0.0, cvg_double dyawfi_t = 0.0, cvg_double dzfi_t = 0.0,
			cvg_double pitchfi_t = 0.0, cvg_double rollfi_t = 0.0);
	void setXReference( cvg_double xci_in);
	void setYReference( cvg_double yci_in);
	void setYawReference( cvg_double yawci_in);
	void setZReference( cvg_double zci_in);
	void setVFIReference( cvg_double vxfi_in, cvg_double vyfi_in);
	void setVXFIReference( cvg_double vxfi_in);
	void setVYFIReference( cvg_double vyfi_in);
	void setPitchRollReference( cvg_double pitchfi_in, cvg_double rollfi_in);
	void setDYawFIReference( cvg_double dyawfi_in);
	void setDZFIReference( cvg_double dzfi_in);
	void setAllReferences2ActualValues();
	// END: ***** Set controller reference functions *****

	// ***** Get/Accessors Controller functions *****
	void getLastOutput( cvg_double *pitchco_out, cvg_double *rollco_out, cvg_double *dyawco_out, cvg_double *dzco_out);
	void getCntReferences( cvg_double *xci_out, cvg_double *yci_out, cvg_double *yawci_out, cvg_double *zci_out);
	// 		***** Scheduler part of MyDrone *****
	SM_stateNames::stateNames getCurrentState();
	int getCheckpoint();
	int getTrueCheckpoint();
	// 		END: ***** Scheduler part of MyDrone *****
	// END: ***** Get/Accessors Controller functions *****

public:
	// ***** Control mode functions *****
	void setControlMode(Controller_MidLevel_controlMode::controlMode mode);
	Controller_MidLevel_controlMode::controlMode getControlMode();
	// 		(BEFORE) ***** Scheduler part of MyDrone *****
public:
	void setTrajectory(TrajectoryType &trajectory, TrajectoryConfiguration traj_config = TrajectoryConfiguration());
private:
	void setPositionControl();
	void setSpeedControl();
	// 		(BEFORE) END: ***** Scheduler part of MyDrone *****
	// END: ***** Control mode functions *****

private:
	// ***** [Control] datalogging *****
	void saveCntRefData2Disk();
	class ControlDataEvent : public DroneProxy::Logs::DataWriter::Event {
	private:
		cvgString log_msg;
	public:
		ControlDataEvent(cvg_ulong timeCode, double elapsed_seconds, SM_stateNames::stateNames sm_state, double x, double y, double vx, double vy, double yaw,
							double z, double vxfi, double vyfi, double dyawfi, double dzfi, double pitchfi, double rollfi);
		virtual inline ~ControlDataEvent() { }
		virtual inline cvgString toString() { return log_msg; }
	};
	// END: ***** [Control] datalogging *****
	// END END END  * Things that have to be changed for each controller **** END END END
};


#endif /* MULTIROTOR_CONTROLLER_H_ */
