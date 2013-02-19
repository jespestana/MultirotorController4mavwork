/*
 * Controller_StateMachine_v2.h
 *
 *  Created on: Nov 27, 2012
 *      Author: jespestana
 */

#ifndef CONTROLLER_STATEMACHINE_V2_H_
#define CONTROLLER_STATEMACHINE_V2_H_

#include <atlante.h>
#include <math.h>

#include "config_Mydrone.h" // Which drone is going to be used?

#include "stateObserver/EKF_lib/matrixLib.h"

#include "Other/jesus_library.h"
#include "Other/Timer.h"

#include "controller/stateMachine/Controller_SM_stateNames.h"
#include "controller/stateMachine/Controller_SM_Trajectory_v2.h"
typedef Controller_SM_Trajectory_v2 TrajectoryType;
typedef Controller_SM_TrajectoryConfiguration TrajectoryConfiguration;

#include "controller/other/FilteredDerivative.h"

class Controller_StateMachine_v2 {
private:

	// ***************************** General State Machine Data *****************************
	// SM_Inputs
	cvg_double xei, yei, zei, vxei, vyei, vzei, yawei, dyawei; 			// ei  ~ estimation input

	// SM_Outputs
	// Note that all this data is passed through the trajectory controller
	//       (even in cases where it, in turn, passes directly the data to the parrot)
	cvg_double vxfo, vyfo;							// fo ~ feedforward output (to Speed controller)
	CVG_BlockDiagram::FilteredDerivative derivBlock_vxfo, derivBlock_vyfo;
	cvg_double dvxfo, dvyfo;	// derivatives of speed references
	cvg_double pitchfo, rollfo, vzfo, dyawfo;		// fo ~ feedforward output (directly to parrot)
	cvg_double xrefo, yrefo, zrefo, yawfo;			// refo ~ (position) reference outputs (to position controller)

	// SM trajectory related information
	TrajectoryType trajectory; // trajectory.traj_config has the trajectory configuration parameters
	cvg_int pr_checkpoint, checkpoint, true_checkpoint;

	// SM state
	bool started;
	SM_stateNames::stateNames current_state;
	bool justChangedState;			// Stores whether a state change has just ocurred

	int debug_counter;


	// ***************************** General State Machine Functions ************************************
public:
	Controller_StateMachine_v2();
	virtual ~Controller_StateMachine_v2();
	cvg_bool reset();
	inline int getCheckpoint() 			{ return checkpoint; }
	inline int getTrueCheckpoint() 		{ return true_checkpoint; }
	const char *getCurrentStateName(SM_stateNames::stateNames state_in);
	inline SM_stateNames::stateNames getCurrentState() 	{ return current_state; }
	void setInputs( cvg_double xei_i, cvg_double yei_i, cvg_double zei_i,
		cvg_double vxei_i, cvg_double vyei_i, cvg_double vzei_i, cvg_double yawei_i, cvg_double dyawei_i);
	void getOutput( cvg_double &xrefo_o, cvg_double &yrefo_o, cvg_double &zrefo_o,
			cvg_double &vxfo_o, cvg_double &vyfo_o, cvg_double &vzfo_o,
			cvg_double &yawfo_o, cvg_double &dyawfo_o,
			cvg_double &pitchfo_o, cvg_double &rollfo_o);

	cvg_bool setTrajectory( TrajectoryType &trajectory, TrajectoryConfiguration traj_config);
	inline TrajectoryType getTrajectory() { return trajectory; }
	cvg_bool activateTrajectoryControl();
	inline void activatePositionControl() 					{ current_state = SM_stateNames::POSITION_CONTROL; }
	inline void activateSpeedControl() 						{ current_state = SM_stateNames::SPEED_CONTROL; }
private:
	void initSM( cvg_double x_act, cvg_double y_act, cvg_double z_act, cvg_double yaw_act);

private:
	void process();
//	void processMandatoryCode();
	void updateTrueCheckpointValue();
	void processPositionControl();
	void processSpeedControl();

	// ***************************** Straight line state: data and functions ****************************
	void processStraight();
	void isStraightFinished();
	SM_stateNames::stateNames r_nextState;
	Vector r_p0, r_ur, r_ur2;		// initial point and direction of straight line (recta)
	cvg_double rs_end;		// distance from p0 that indicates the end of the straight line (recta)
	cvg_double rs_act;		// actual value of s;


	// ***************************** Perform turn state: data and functions *****************************
	Vector 		c_pinit, c_pc, c_pend;	// Initial point of turn
	cvg_double 	c_alim; 		// limit yaw angle from pinit to "pend" that indicates the end of the turn
	Vector		c_u0;			// +1: clockwise turn, -1: counter_clockwise turn
	cvg_double  c_vc, c_Rt; 	// Speed reference during turn and radius of turn circle
	bool 		c_changeState;
	SM_stateNames::stateNames  c_nextState;
	void processTurn();			// "during:"
	void isTurnFinished();

	// ***************************** Hover to checkpoint state: data and functions **********************
	// This state only uses the information of "checkpoint"
	void processHover();		// "during:"
	void isHoverFinished();		// transition
	Vector h_checkpoint; // dim3
	cvg_bool h_stay_in_last_checkpoint;

};

#endif /* CONTROLLER_STATEMACHINE_V2_H_ */
