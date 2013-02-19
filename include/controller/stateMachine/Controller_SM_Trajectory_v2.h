/*
 * Controller_SM_Trajectory_v2.h
 *
 *  Created on: Nov 27, 2012
 *      Author: jespestana
 */

#ifndef CONTROLLER_SM_TRAJECTORY_V2_H_
#define CONTROLLER_SM_TRAJECTORY_V2_H_

#include "config_Mydrone.h"
#include <atlante.h>
#include "stateObserver/EKF_lib/matrixLib.h"
#include "Other/jesus_library.h"
#include <iostream>

//#ifndef SM_STATEMACHINE_CONFIDENCE_RADIUS
//#define SM_STATEMACHINE_CONFIDENCE_RADIUS 	0.25
//#define SM_RTMIN							0.20
//#define SM_RTMAX							40.0
//#define SM_TR_V	 1.7
//#define SM_SPEEDPLAN_DELTACHECKPOINT 5
//#endif

class Controller_SM_TrajectoryWaypoint {
public:
	cvg_double x, y, z;
	cvg_bool achieved_checkpoint, achieved_true_checkpoint; // keep track of trajectory progress

	Controller_SM_TrajectoryWaypoint(cvg_double x, cvg_double y, cvg_double z);
////////	Controller_SM_TrajectoryWaypoint() { x = 0;  y = 0;  z = 0; }
	virtual ~Controller_SM_TrajectoryWaypoint();
	void convert2Vector(Vector& lv);
public:
	/* ***** Controller_SM_TrajectoryWaypoint - Trajectory planning ***** */
	// SM_stateNames::STRAIGHT mode parameters
	cvg_double L, s_end;
	// SM_stateNames::TURN mode parameters
	cvg_double R, vc, alim;
	Vector u0, r_ur;
	/* END: ***** Controller_SM_TrajectoryWaypoint - Trajectory planning ***** */
};


class Controller_SM_TrajectoryConfiguration {
public:
	cvg_double chk_clearance_R;
	cvg_double chk_R, Rt_min, Rt_max;
	cvg_double vmax_xy, vmax_z;
	cvg_double vstall_turn, amax, num_chk_speed_plan, speed_tr;
	cvg_double straighmode_safetyzone_radius_m, turnmode_safetyzone_radius_m,
			turnmode_safetyzone_altitude_m, turnmode_safetyzone_negalpha_rad;

	Controller_SM_TrajectoryConfiguration() { set2defaultValues(); }
	void set2defaultValues();
};


#include <deque>
typedef std::deque<Controller_SM_TrajectoryWaypoint> trajectory_container;
//#include <vector>
//typedef std::vector<Controller_SM_TrajectoryWaypoint> trajectory_container;

class Controller_SM_Trajectory_v2 {
private:
	trajectory_container waypoints;
	cvg_int initial_checkpoint;
	cvg_bool isPeriodic;
	Controller_SM_TrajectoryWaypoint error_waypoint;
//	SM_stateNames::stateNames initial_state; // I think that this is not useful.

public:
	Controller_SM_Trajectory_v2();
	Controller_SM_Trajectory_v2(cvg_double x_0, cvg_double y_0, cvg_double z_0);
	Controller_SM_Trajectory_v2(cvg_double *routeX, cvg_double *routeY, cvg_double *routeZ, cvg_int num_waypoints);
	virtual ~Controller_SM_Trajectory_v2();

	cvg_bool check();	// check that internal variables are consistent: initial_checkpoint, length >= 1.
	cvg_bool reset();	// prepare trajectory to be introduced into the state machine

	cvg_bool setInitialCheckpoint(cvg_int initial_checkpoint);
	inline void setPeriodic(cvg_bool isPeriodic) 					{ this->isPeriodic = isPeriodic; }
	inline cvg_int getInitialCheckpoint()					 		{ return initial_checkpoint; }
	cvg_int incrementCheckpoint(cvg_int checkpoint);
	cvg_int decrementCheckpoint(cvg_int checkpoint);
	cvg_bool routeFinished(cvg_int checkpoint);
	inline cvg_bool getIsPeriodic()		 							{ if ( getLength() < 2 ) isPeriodic = false;  return isPeriodic; }
	inline int getLength()		 									{ return waypoints.size(); }
	inline cvg_bool empty()											{ return waypoints.empty(); }
	inline cvg_bool checkpointValueIsValid(cvg_int chk)		{ return ( (chk >= 0) && (chk < getLength()) ); }
	inline void achievedCheckpoint(cvg_int chk)				{ if (checkpointValueIsValid(chk)) waypoints[chk].achieved_checkpoint = true; }
	inline void achievedTrueCheckpoint(cvg_int t_chk)		{ if (checkpointValueIsValid(t_chk)) waypoints[t_chk].achieved_true_checkpoint = true; }
//private:
	inline cvg_bool isAchievedCheckpoint(cvg_int chk)		{ if (checkpointValueIsValid(chk)) return waypoints[chk].achieved_checkpoint;  else  return false;}
	inline cvg_bool isAchievedTrueCheckpoint(cvg_int t_chk){ if (checkpointValueIsValid(t_chk)) return waypoints[t_chk].achieved_true_checkpoint;  else  return false;}
public:
	Controller_SM_TrajectoryWaypoint& operator[](cvg_int i);
	// int operator==(const Controller_SM_Trajectory_v2& right) const;  // I think this might not be necessary.

	void addWaypoint(cvg_double x, cvg_double y, cvg_double z);
	cvg_bool deleteLastWaypoint();
	void deleteFirstWaypoints(int until_checkpoint = 0);
//	inline void clear() 											{ waypoints_my_clear();  initial_checkpoint = 0;  isPeriodic = false;}
	void clear();

	/* ***** Controller_SM_Trajectory_v2 - Trajectory planning ***** */
	// planify trajectory from chk = checkpoint to chk = last_checkpoint "as if periodic"
public:
	Controller_SM_TrajectoryConfiguration traj_config;
	cvg_bool planify_trajectory(Controller_SM_TrajectoryConfiguration *configuration, cvg_int segment = 1, cvg_bool initialize_planning_variables = false);
	cvg_bool calculate_straight( cvg_int segment, Vector &r_p0, Vector &r_ur, Vector &r_ur2, cvg_double &rs_end, cvg_double &c_Rt);
	void calculate_turn(cvg_int turn, Vector &c_pinit, Vector &c_pend, Vector &c_pc, cvg_double &c_alim, Vector &c_u0);
	cvg_double obtainPlannedSpeed( cvg_int chk, cvg_double x, cvg_double y, cvg_double z, cvg_double v_current);
private:
	void initializeAuxiliarPlanningVariables();
	void destroyAuxiliarPlanningVariables();
	cvg_bool deleteWaypointInPlanning(cvg_int checkpoint);

	// planning variables
	Vector aux_vector;
	cvg_bool pr_isPeriodic;
	Vector r_p0, r_p1, r_p2;
	Vector r_ur, r_ur2;
	cvg_double rs_end;
	cvg_double c_Rt, c_vc;
	Vector c_u0;
	/* END: ***** Controller_SM_Trajectory_v2 - Trajectory planning ***** */
};

#endif /* CONTROLLER_SM_TRAJECTORY_V2_H_ */
