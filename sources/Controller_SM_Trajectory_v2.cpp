/*
 * Controller_SM_Trajectory_v2.cpp
 *
 *  Created on: Nov 27, 2012
 *      Author: jespestana
 */

#include "controller/stateMachine/Controller_SM_Trajectory_v2.h"

/* ***** Controller_SM_TrajectoryWaypoint functions ***** */

// DONE: solve - Problema de glibc:
// 	  1) double free or corruption (fasttop)
//    2) habia otro que ponia corrupted linked list o algo asi que no consigo volver a provocar
//  3) poniendo  u0(0), r_ur(0) en el conscrutor initialization list no me pasa el problema.
// Posibles soluciones en internet:
//  http://cboard.cprogramming.com/c-programming/111240-double-free-corruption-fasttop.html
//  http://stackoverflow.com/questions/2403020/why-doesnt-the-c-default-destructor-destroy-my-objects
//  http://stackoverflow.com/questions/106508/what-is-a-smart-pointer-and-when-should-i-use-one
// Por lo que he leido puede ser que tengo que definir el comando delete y new para la clase Vector. Dbe ser que hay veces que
// se llama solo al destructor y otras que se llama a delete y al destructor causando doble destruccion... pero no estoy seguro.
// A parte, creo que es un problema con objetos temporales de los que crea C++.
// Dandole mas vueltas creo que es un problema con las shallow copies de JL.
// Relacionados con el problema de shallow copy:
//  - http://stackoverflow.com/questions/7297024/call-default-copy-constructor-from-within-overloaded-copy-constructor
//  - http://stackoverflow.com/questions/3279543/what-is-the-copy-and-swap-idiom
//  - http://stackoverflow.com/questions/4172722/what-is-the-rule-of-three
//  - http://en.wikipedia.org/wiki/Single_responsibility_principle ; "Sometimes you need to implement a class that manages
//     a resource. (Never manage multiple resources in a single class, this will only lead to pain.) "
Controller_SM_TrajectoryWaypoint::Controller_SM_TrajectoryWaypoint(cvg_double x, cvg_double y, cvg_double z) : u0(3), r_ur(3) {
	this->x = x;
	this->y = y;
	this->z = z;

	achieved_checkpoint = false;
	achieved_true_checkpoint = false;

	// default "empty" values
	// SM_stateNames::STRAIGHT mode parameters
	L = 0.0;
	s_end = 0.0;
//	r_ur(3);

	// SM_stateNames::TURN mode parameters
	R = 0.0;
	vc = 0.0;
	alim = 0.0;
//	u0(3);
}

Controller_SM_TrajectoryWaypoint::~Controller_SM_TrajectoryWaypoint() {
}

void Controller_SM_TrajectoryWaypoint::convert2Vector(Vector& lv) {
	if ( lv.length() != 3 ) {
		lv.deletion();
		lv.creation(3);
	}
	lv.setValueData(this->x,1);
	lv.setValueData(this->y,2);
	lv.setValueData(this->z,3);
}

/* END: ***** Controller_SM_TrajectoryWaypoint functions ***** */

/* ***** Controller_SM_TrajectoryConfiguration functions ***** */
void Controller_SM_TrajectoryConfiguration::set2defaultValues() {

	chk_clearance_R		= SM_CHECKPOINT_CLEARANCE_RADIUS;

	chk_R   	= SM_TRAJPLAN_CHECKPOINT_RADIUS;
	Rt_min  	= SM_TRAJPLAN_RTMIN;
	Rt_max  	= SM_TRAJPLAN_RTMAX;

	vmax_xy 	= MULTIROTOR_TRAJECTORYCONTROLLER_VXY_AT_MAX;
	vmax_z  	= MULTIROTOR_TRAJECTORYCONTROLLER_VZ_AT_MAX;

	vstall_turn 		= SM_SPEEDPLAN_STALLTURN_VMAX;
	amax   				= SM_SPEEDPLAN_AMAX;
	num_chk_speed_plan 	= SM_SPEEDPLAN_DELTACHECKPOINT;
	speed_tr 			= SM_SPEEDPLAN_SPEED_TR;

	straighmode_safetyzone_radius_m = SM_STRAIGHTMODE_SAFETYZONE_THRESHOLD;
	turnmode_safetyzone_radius_m	= SM_TURNMODE_RADIUS_SAFETYZONE_THRESHOLD;
	turnmode_safetyzone_altitude_m	= SM_TURNMODE_ALTITUDE_SAFETYZONE_THRESHOLD;
	turnmode_safetyzone_negalpha_rad= SM_TURNMODE_NEGALPHA_SAFETYZONE_THRESHOLD;
}
/* END: ***** Controller_SM_TrajectoryConfiguration functions ***** */

/* ***** Controller_SM_Trajectory_v2 functions ***** */

Controller_SM_Trajectory_v2::Controller_SM_Trajectory_v2() : error_waypoint(0.0, 0.0, -1000.0) {
	initial_checkpoint = 0;
	isPeriodic 		   = false;
}

Controller_SM_Trajectory_v2::Controller_SM_Trajectory_v2(cvg_double x_0, cvg_double y_0, cvg_double z_0) : error_waypoint(0.0, 0.0, -1000.0) {
	initial_checkpoint = 0;
	isPeriodic 		   = false;
	this->addWaypoint( x_0, y_0, z_0);
}

Controller_SM_Trajectory_v2::Controller_SM_Trajectory_v2(cvg_double *routeX, cvg_double *routeY, cvg_double *routeZ, cvg_int num_waypoints) : error_waypoint(0.0, 0.0, -1000.0) {
	initial_checkpoint = 0;
	isPeriodic 		   = false;
	for (cvg_int i = 0; i<num_waypoints; i++) {
		this->addWaypoint( routeX[i], routeY[i], routeZ[i]);
	}
}

Controller_SM_Trajectory_v2::~Controller_SM_Trajectory_v2() {
	this->clear();
}

cvg_bool Controller_SM_Trajectory_v2::check() {
	return setInitialCheckpoint( initial_checkpoint );
}

cvg_bool Controller_SM_Trajectory_v2::reset() {
	cvg_bool error_ocurred = this->check();

	for (int i = 0; i < (this->getInitialCheckpoint()-1); i++) {
		waypoints[i].achieved_checkpoint = true;
		waypoints[i].achieved_true_checkpoint = true;
	}

	return error_ocurred;
}

cvg_bool Controller_SM_Trajectory_v2::setInitialCheckpoint(cvg_int initial_checkpoint) {
	cvg_bool error_ocurred = false;

	cvg_int N = getLength();
	if (initial_checkpoint > (N-1))
		initial_checkpoint = N-1;

	if (initial_checkpoint < 0)
		initial_checkpoint = 0;

	if ( N == 0 )
		error_ocurred = true; // trajectory is empty

	this->initial_checkpoint = initial_checkpoint;
	return error_ocurred;
}

cvg_int Controller_SM_Trajectory_v2::incrementCheckpoint(cvg_int checkpoint) {

	cvg_int N = getLength();

	if ( checkpoint < 0 ) {
		checkpoint = 0;
		return checkpoint;
	}

	if (!isPeriodic) {
		if ( (checkpoint < N-1) )
			return jesus_library::fmod( checkpoint+1, N);
	} else { // isPeriodic
		return jesus_library::fmod( checkpoint+1, N);
	}

	return checkpoint;
}

cvg_int  Controller_SM_Trajectory_v2::decrementCheckpoint(cvg_int checkpoint) {

	cvg_int N = getLength();

	if ( checkpoint < 0 ) {
		checkpoint = 0;
		return checkpoint;
	}

	if (!isPeriodic) {
		if ( (checkpoint > 0 ) )
			return jesus_library::fmod( checkpoint-1, N);
		else    // checkpoint == 0
			return checkpoint;
	} else {    // isPeriodic
		return jesus_library::fmod( checkpoint-1, N);
	}

	return checkpoint;
}

cvg_bool Controller_SM_Trajectory_v2::routeFinished(cvg_int checkpoint) {

	if (isPeriodic)
		return false;
	else {
		cvg_int last_checkpoint = (this->getLength() - 1);
//		std::cout << "(chk == last_chk): " << ( checkpoint == last_checkpoint ) <<
//					" achieved(lst_chk): " << isAchievedCheckpoint(last_checkpoint) <<
//					" achievedTrue(lst_chk): " << isAchievedTrueCheckpoint(last_checkpoint) << std::endl;
		if ( ( checkpoint == last_checkpoint ) && isAchievedCheckpoint(last_checkpoint) && isAchievedTrueCheckpoint(last_checkpoint) )
			return true;
		else
			return false;
	}
}

Controller_SM_TrajectoryWaypoint& Controller_SM_Trajectory_v2::operator[](cvg_int i) {
	if (this->empty()) {
		return error_waypoint; // waypoint_error
//		Controller_SM_TrajectoryWaypoint waypoint_error( 0.0, 0.0, -1.0);
//		return waypoint_error;
	}
//	if (isPeriodic) {
//		cvg_int index = i % this->getLength();
//		index = index > 0 ? index : index + this->getLength();
//		return waypoints[index];
//	} else {
//		if (i<0)
//			return waypoints[0];
//		if (i >= (int) this->getLength())
//			return waypoints[this->getLength()-1];
//		return waypoints[i];
//	}

	cvg_int index = jesus_library::fmod( i, this->getLength());
	return waypoints[index];
}

void Controller_SM_Trajectory_v2::addWaypoint(cvg_double x, cvg_double y, cvg_double z) {
	waypoints.push_back(Controller_SM_TrajectoryWaypoint( x, y, z));
	check();
}

cvg_bool Controller_SM_Trajectory_v2::deleteLastWaypoint() {
	cvg_bool error_ocurred = false;
	if (this->empty()) {
		error_ocurred = true;
		return error_ocurred;
	}

	if (this->getLength() > 1)
		waypoints.pop_back();
	check();
	error_ocurred = false;
	return error_ocurred;
}

void Controller_SM_Trajectory_v2::deleteFirstWaypoints(int until_checkpoint) {
	if (until_checkpoint < 0)
		return;
	if (until_checkpoint > (int) (this->getLength()-1) )
		return;

	waypoints.erase( waypoints.begin(), waypoints.begin()+(until_checkpoint+1));
	check();
}

void Controller_SM_Trajectory_v2::clear() {
	waypoints.clear();
	initial_checkpoint = 0;
	isPeriodic = false;
}

cvg_bool Controller_SM_Trajectory_v2::planify_trajectory(Controller_SM_TrajectoryConfiguration *config, cvg_int segment, cvg_bool func_input_initialize_planning_variables) {
	// This is a recursive function that just to be called:
	//		- planify_trajectory(&config); // to planify whole trajectory with default checkpoint radius
	//		- planify trajectory(&config, segment_number, true); // planify trajectory from segment#segment_number onwards
	//		- the trajectory is always planified as if it was periodic (segment#(N-1) goes from chk#(N-1) to chk#0

//	std::cout << "Controller_SM_Trajectory_v2::planify_trajectory( segment = " << segment << " )" << std::endl;

	cvg_bool error_ocurred;

	cvg_int N = getLength();
	if (N < 1) // error, trayectoria vacia
		return (error_ocurred = true);

	if ( segment < 1)
		return planify_trajectory( config, 1);

	cvg_bool initialize_planning_variables;
	if ( (segment == 1) || func_input_initialize_planning_variables ) // DO: initialize_planning_variables
		initializeAuxiliarPlanningVariables();
	else // DO NOT: initialize_planning_variables (do nothing)
		initialize_planning_variables = false;

	if ( N == 1 ) {
		cvg_int target_checkpoint = 0;
		waypoints[target_checkpoint].L     = -1.0;
		waypoints[target_checkpoint].s_end = -1.0;
		waypoints[target_checkpoint].R     = -1.0;
		waypoints[target_checkpoint].vc    = -1.0;
		waypoints[target_checkpoint].alim  = -1.0;
		if (waypoints[target_checkpoint].u0.length() != 3)
			waypoints[target_checkpoint].u0.creation(3);
		waypoints[target_checkpoint].u0.setValueData( 1.0, 3);
		if (waypoints[target_checkpoint].r_ur.length() != 3)
			waypoints[target_checkpoint].r_ur.creation(3);
		waypoints[target_checkpoint].r_ur.setValueData( 1.0, 3);
		traj_config = *config;
		error_ocurred = false;
		return error_ocurred;
	}

	// 1º) Calculate r_p0, r_p1, r_p2, r_ur, r_ur2, L, L2, rs_end <-si_no_hay_curva
	cvg_double L, L2;
	(*this)[segment-1].convert2Vector(r_p0);
	(*this)[segment].convert2Vector(r_p1);
	(*this)[segment+1].convert2Vector(r_p2);

//	jesus_library::unitarizeVector(r_ur);
	L  = jesus_library::unitaryVectorFrom2Points( r_ur, r_p0, r_p1);
	if ( L < config->chk_R ) {
		cvg_bool avoided2delete_waypoint0 = deleteWaypointInPlanning(segment);
//		return planify_trajectory( config, segment); // This case only occurs when segment == 1
		return planify_trajectory( config, 1);       // This case only occurs when segment == 1
	}

	L2 = jesus_library::unitaryVectorFrom2Points( r_ur2, r_p1, r_p2);
	if ( L2 < config->chk_R ) {
		cvg_bool avoided2delete_waypoint0 = deleteWaypointInPlanning(incrementCheckpoint(segment));
		if (avoided2delete_waypoint0)
			return planify_trajectory( config, segment-1);
		else
			return planify_trajectory( config, segment);
	}

	rs_end = L - config->chk_R; // si no hay curva

	// 3º) Calcular c_u0, c_alim
	jesus_library::crossProduct( c_u0, r_ur, r_ur2);
	jesus_library::unitarizeVector( c_u0);

	cvg_double cos_alpha = jesus_library::dotProduct( r_ur, r_ur2);
	cos_alpha = jesus_library::saturate( cos_alpha, -1.0, +1.0);
	cvg_double alpha = M_PI - acos(cos_alpha);
	cvg_double d = config->chk_R/2.0;

	// 2º) Calcular c_Rt, c_vc
	c_Rt = d*sin(alpha/2.0)/(1-sin(alpha/2.0));

	if ( (c_Rt < config->Rt_min) || (c_Rt > config->Rt_max) ) {
		if (c_Rt < config->Rt_min)
			c_vc = config->vstall_turn;
		if (c_Rt > config->Rt_max)
			c_vc = jesus_library::calculateVmax( r_ur, config->vmax_xy, config->vmax_z);
		c_Rt = -1.0;
		rs_end = L - config->chk_R;
	} else {
		c_vc = pow( c_Rt*config->amax, 0.5);
		cvg_double vaux = jesus_library::calculateVmax( r_ur, config->vmax_xy, config->vmax_z);
		if (c_vc > vaux)
			c_vc = vaux;
		vaux = jesus_library::calculateVmax( r_ur2, config->vmax_xy, config->vmax_z);
		if (c_vc > vaux)
			c_vc = vaux;
		rs_end = L - (c_Rt+d)*cos(alpha/2.0);
	}

	// 4º) Copy turn parameters to waypoint data
	cvg_int target_checkpoint = jesus_library::fmod( segment, N);
	waypoints[target_checkpoint].L  = L;
	waypoints[target_checkpoint].s_end = rs_end;
	if (waypoints[target_checkpoint].r_ur.length() != 3)
		waypoints[target_checkpoint].r_ur.creation(3);
	waypoints[target_checkpoint].r_ur.copy(&r_ur);
	waypoints[target_checkpoint].R  = c_Rt;
	waypoints[target_checkpoint].vc = c_vc;
	waypoints[target_checkpoint].alim = M_PI - alpha;
	if (waypoints[target_checkpoint].u0.length() != 3)
		waypoints[target_checkpoint].u0.creation(3);
	waypoints[target_checkpoint].u0.copy(&c_u0);

	if ( segment > (N-1) ) {
		destroyAuxiliarPlanningVariables();
		traj_config = (*config);
		error_ocurred = false;
		return error_ocurred;
	} else { // then recursive function call
		return planify_trajectory( config, segment+1);
	}
}

cvg_bool Controller_SM_Trajectory_v2::calculate_straight( cvg_int segment, Vector &r_p0, Vector &r_ur, Vector &r_ur2, cvg_double &rs_end, cvg_double &c_Rt) {
	segment = jesus_library::fmod( segment, this->getLength() );
	// retrieve r_p0
	(*this)[segment-1].convert2Vector(r_p0);
	(*this)[segment].convert2Vector(r_p1);
	(*this)[segment+1].convert2Vector(r_p2);
	// r_ur, r_ur2
	jesus_library::unitaryVectorFrom2Points( r_ur, r_p0, r_p1);
	jesus_library::unitaryVectorFrom2Points( r_ur2, r_p1, r_p2);
	// rs_end
	rs_end = (*this)[segment].s_end;
	// c_Rt
	c_Rt = (*this)[segment].R;

	cvg_int N = getLength();
	cvg_bool is_last_checkpoint; // tells the caller whether this is the last straight segment
	if ( isPeriodic )
		is_last_checkpoint = false;
	else
		if ( segment == N-1 )
			is_last_checkpoint = true;
		else
			is_last_checkpoint = false;
	return is_last_checkpoint;
}

void Controller_SM_Trajectory_v2::calculate_turn(cvg_int turn, Vector &c_pinit, Vector &c_pend, Vector &c_pc, cvg_double &c_alim, Vector &c_u0) {
	turn = jesus_library::fmod( turn, this->getLength() );
	(*this)[turn-1].convert2Vector(r_p0);
	(*this)[turn].convert2Vector(r_p1);
	(*this)[turn+1].convert2Vector(r_p2);
	jesus_library::unitaryVectorFrom2Points( r_ur, r_p0, r_p1);
	jesus_library::unitaryVectorFrom2Points( r_ur2, r_p1, r_p2);

	// c_alim, c_u0
	c_alim = (*this)[turn].alim;
	c_u0.copy( &( (*this)[turn].u0 ) );

	// c_pinit
	aux_vector.copy(&r_ur);
	jesus_library::multiplyDoubleVsVector( (*this)[turn].s_end, aux_vector);
	c_pinit.addition( &r_p0, &aux_vector);

	// c_pend
	aux_vector.copy(&r_ur2);
	jesus_library::multiplyDoubleVsVector( (*this)[turn].L - (*this)[turn].s_end, aux_vector);
	c_pend.addition( &r_p1, &aux_vector);

	// c_pc
	jesus_library::crossProduct( aux_vector, c_u0, r_ur);
	jesus_library::multiplyDoubleVsVector( (*this)[turn].R, aux_vector);
	c_pc.addition( &c_pinit, &aux_vector);
}

cvg_double Controller_SM_Trajectory_v2::obtainPlannedSpeed( cvg_int chk, cvg_double x, cvg_double y, cvg_double z, cvg_double v_current) {
	chk = jesus_library::fmod( chk, getLength() );
	cvg_double v_max  = jesus_library::calculateVmax( waypoints[chk].r_ur, traj_config.vmax_xy, traj_config.vmax_z);
	cvg_double v_plan = v_max, v_plani = v_max;

	cvg_double cummL = 0.0, dL_turn = 0.0, dL_trv = 0.0;
	cvg_double cummL_exit = ( v_current*v_current/(2*traj_config.amax) ) * 10.0;
	cvg_bool first_iteration = true;
	cvg_int  i_aux;

	for ( int i = chk; i < chk + traj_config.num_chk_speed_plan; i++ ) {
		i_aux = jesus_library::fmod( i, getLength());
		if ( first_iteration ) {
			cummL = pow( pow( waypoints[chk].x - x, 2) + pow( waypoints[chk].y - y, 2) + pow( waypoints[chk].z - z, 2), 0.5);
			first_iteration = false;
		} else {
			cummL += waypoints[i_aux].L;
		}
		if ( waypoints[i_aux].R > 0 )
			dL_turn = ( waypoints[i_aux].R + traj_config.chk_R/2.0 )*sin( waypoints[i_aux].alim/2.0 );
		else
			dL_turn = 0.0;
		dL_trv = traj_config.speed_tr*fabs(v_current);

		cvg_double aux_cummL = (cummL - dL_turn - dL_trv);
		if ( aux_cummL < 0 )
			v_plani = fabs(waypoints[i_aux].vc);
		else
			v_plani = pow( pow(fabs(waypoints[i_aux].vc),2) + 2*traj_config.amax*aux_cummL, 0.5);

		if ( v_plani < v_plan )
			v_plan = v_plani;

		if ( aux_cummL > cummL_exit )
			break;
	}

	first_iteration = true;
	i_aux = jesus_library::fmod( chk-1, getLength());
	for ( int i = i_aux; i > (chk-1) - traj_config.num_chk_speed_plan; i--) {
		i_aux = jesus_library::fmod( i, getLength());

		if ( first_iteration ) {
			cummL = pow( pow( waypoints[i_aux].x - x, 2) + pow( waypoints[i_aux].y - y, 2) + pow( waypoints[i_aux].z - z, 2), 0.5);
			first_iteration = false;
		} else {
			cummL += waypoints[i_aux].L;
		}

		if ( waypoints[i_aux].R > 0 )
			dL_turn = ( waypoints[i_aux].R + traj_config.chk_R/2.0 )*sin( waypoints[i_aux].alim/2.0 );
		else
			dL_turn = 0.0;
//		dL_trv = traj_config.speed_tr*fabs(v_current);
//		cvg_double aux_cummL = (cummL - dL_turn - dL_trv);
		dL_trv = 0.0;
		cvg_double aux_cummL = (cummL - dL_turn);
//		cvg_double aux_cummL = cummL;
		if ( aux_cummL < 0 )
			v_plani = fabs(waypoints[i_aux].vc);
		else
			v_plani = pow( pow(fabs(waypoints[i_aux].vc),2) + 2*traj_config.amax*aux_cummL, 0.5);
//		v_plani = pow( pow(fabs(waypoints[i_aux].vc),2) + 2*traj_config.amax*cummL, 0.5);

		if ( v_plani < v_plan )
			v_plan = v_plani;

		if ( aux_cummL > cummL_exit )
			break;
	}

	return v_plan;
}

cvg_bool Controller_SM_Trajectory_v2::deleteWaypointInPlanning(cvg_int checkpoint) {

	cvg_bool avoided2delete_waypoint0;

	if ( checkpoint == 0 ) {
		deleteLastWaypoint();
		avoided2delete_waypoint0 = true;
	} else {
		waypoints.erase( waypoints.begin() + checkpoint );
		avoided2delete_waypoint0 = false;
	}
	return avoided2delete_waypoint0;
}

void Controller_SM_Trajectory_v2::initializeAuxiliarPlanningVariables() {
	pr_isPeriodic = isPeriodic;
	isPeriodic = true;

	aux_vector.creation(3);
	r_p0.creation(3); r_p1.creation(3); r_p2.creation(3);
	r_ur.creation(3); r_ur2.creation(3);
	c_u0.creation(3);
	rs_end = -1.0;
	c_Rt =   -1.0;
	c_vc =    0.0;
}

void Controller_SM_Trajectory_v2::destroyAuxiliarPlanningVariables() {
	isPeriodic 	  = pr_isPeriodic;
	pr_isPeriodic = false;

//	aux_vector.deletion();
//	r_p0.deletion(); r_p1.deletion(); r_p2.deletion();	// used in calculate_straight(...)
//	r_ur.deletion(); r_ur2.deletion();					// used in calculate_turn(...)
	c_u0.deletion();
	rs_end = -1.0;
	c_Rt =   -1.0;
	c_vc =    0.0;
}

/* END: ***** Controller_SM_Trajectory_v2 functions ***** */

