/*
 * Controller_StateMachine_v2.cpp
 *
 *  Created on: Nov 27, 2012
 *      Author: jespestana
 */

#include "controller/stateMachine/Controller_StateMachine_v2.h"

// ***************************** General State Machine Functions ************************************
Controller_StateMachine_v2::Controller_StateMachine_v2() :
						r_p0(3), r_ur(3), r_ur2(3), r_nextState(SM_stateNames::HOVER),
						c_pc(3), c_u0(3), c_pinit(3), c_pend(3),
						c_changeState(false), c_nextState(SM_stateNames::HOVER),
						trajectory(),
						started(false),
						derivBlock_vxfo(), derivBlock_vyfo() {

		current_state = MULTIROTOR_INIT_CONTROLMODE_SM;
		justChangedState = true;

		h_stay_in_last_checkpoint = false;

		debug_counter = 0;


		derivBlock_vxfo.setResponseTime( SM_TRAJECTORYMODE_TILTFO_TR );
		derivBlock_vxfo.reset();
		derivBlock_vyfo.setResponseTime( SM_TRAJECTORYMODE_TILTFO_TR );
		derivBlock_vyfo.reset();
}

Controller_StateMachine_v2::~Controller_StateMachine_v2() {}

inline const char *Controller_StateMachine_v2::getCurrentStateName(SM_stateNames::stateNames state_in) {
	switch (state_in) {
	case SM_stateNames::STRAIGHT:
		return "STRAIGHT";
		break;
	case SM_stateNames::TURN:
		return "TURN";
		break;
	case SM_stateNames::HOVER:
		return "HOVER";
		break;
	case SM_stateNames::SPEED_CONTROL:
		return "SPEED_CONTROL";
		break;
	case SM_stateNames::POSITION_CONTROL:
		return "POSITION_CONTROL";
		break;
	}
	return "unknown";
}

void Controller_StateMachine_v2::setInputs( cvg_double xei_i, cvg_double yei_i, cvg_double zei_i,
		cvg_double vxei_i, cvg_double vyei_i, cvg_double vzei_i, cvg_double yawei_i, cvg_double dyawei_i) {
	xei = xei_i;
	yei = yei_i;
	zei = zei_i;
	vxei = vxei_i;
	vyei = vyei_i;
	vzei = vzei_i;
	yawei  = yawei_i;
	dyawei = dyawei_i;
}

void Controller_StateMachine_v2::getOutput( cvg_double &xrefo_o, cvg_double &yrefo_o, cvg_double &zrefo_o,
		cvg_double &vxfo_o, cvg_double &vyfo_o, cvg_double &vzfo_o,
		cvg_double &yawfo_o, cvg_double &dyawfo_o,
		cvg_double &pitchfo_o, cvg_double &rollfo_o) {

	if ( !started ) {
		initSM( xei, yei, zei, yawei);
		started = true;
	} else {
		process();
	}

	xrefo_o   = xrefo;
	yrefo_o   = yrefo;
	zrefo_o   = zrefo;
	yawfo_o   = yawfo;
	vxfo_o    = vxfo;
	vyfo_o    = vyfo;
	vzfo_o    = vzfo;
	dyawfo_o  = dyawfo;
	pitchfo_o = pitchfo;
	rollfo_o  = rollfo;
}

void Controller_StateMachine_v2::process() {

	// Los datos de entrada a la maquina de estados se fijan desde fuera de manera asíncrona
	// La state machine se ejecuta dentro de un controller mutex para que no haya corrupción de datos
//	processMandatoryCode();

	switch ( current_state ) {
	case SM_stateNames::POSITION_CONTROL:
		processPositionControl();
		break;
	case SM_stateNames::SPEED_CONTROL:
		processSpeedControl();
		break;
	case SM_stateNames::STRAIGHT:
		if ( justChangedState ) { // state "entry" functions here
			cvg_int segment = checkpoint, N = trajectory.getLength();
			segment = jesus_library::fmod( segment, N);
			cvg_bool last_checkpoint = trajectory.calculate_straight( segment, r_p0, r_ur, r_ur2, rs_end, c_Rt);
			if ( c_Rt < 0 ) {
				if (last_checkpoint)
					r_nextState = SM_stateNames::HOVER;
				else
					r_nextState = SM_stateNames::STRAIGHT;
			} else {
				if (last_checkpoint)
					r_nextState = SM_stateNames::HOVER;
				else
					r_nextState = SM_stateNames::TURN;
			}
			justChangedState = false;
		}
		processStraight();
		isStraightFinished();
		updateTrueCheckpointValue();
		break;
	case SM_stateNames::TURN:
		if ( justChangedState ) { // state "entry" functions here
			cvg_int turn = pr_checkpoint, N = trajectory.getLength();
			turn = jesus_library::fmod( turn, N);
			trajectory.calculate_turn( turn, c_pinit, c_pend, c_pc, c_alim, c_u0);
			c_vc = trajectory[turn].vc;
			justChangedState = false;
		}
		processTurn();
		isTurnFinished();
		updateTrueCheckpointValue();
		break;
	case SM_stateNames::HOVER:
		justChangedState = false; 	// no "entry" functions in this state
		processHover();
		isHoverFinished();
		updateTrueCheckpointValue();
		break;
	}


	#ifdef SM_STATEMACHINE_DEBUG_OLD
	debug_counter++;
	std::cout << "debug counter    =   " << debug_counter << "\n";
	std::cout << "current position = [ " << xei <<", "<< yei <<", "<< zei << "]\n";
	std::cout << "current state    =   " << getCurrentState() << "; checkpoint = "<< checkpoint << "; true_checkpoint = "<< true_checkpoint <<"\n";
	std::cout << "position command = [ " << xrefo <<", "<< yrefo <<", "<< zrefo << "]\n";
	std::cout << "speed command    = [ " << vxfo <<", "<<  vyfo <<", "<<  vzfo << "]\n\n";
	#endif // SM_STATEMACHINE_DEBUG
}

//void Controller_StateMachine_v2::processMandatoryCode() {
//}

void Controller_StateMachine_v2::updateTrueCheckpointValue() {

	cvg_double xref = trajectory[true_checkpoint].x;
	cvg_double yref = trajectory[true_checkpoint].y;
	cvg_double zref = trajectory[true_checkpoint].z;

	cvg_double dist2checkpoint = sqrt( pow( (xei - xref), 2) + pow( (yei - yref), 2) + pow( (zei - zref), 2) );

	if ( dist2checkpoint <= trajectory.traj_config.chk_clearance_R) {
		trajectory.achievedTrueCheckpoint(true_checkpoint);
		true_checkpoint = trajectory.incrementCheckpoint(true_checkpoint);
	}
}

cvg_bool Controller_StateMachine_v2::setTrajectory( TrajectoryType &trajectory, TrajectoryConfiguration traj_config) {
	cvg_bool error_ocurred = false;

	// Ensure that the trajectory is well configured
	error_ocurred = trajectory.reset();
//	if (error_ocurred)
//		return error_ocurred;

	if ( traj_config.chk_R <= 0 ) {
		traj_config.set2defaultValues();
	}
	error_ocurred = trajectory.planify_trajectory(&traj_config);
	if (error_ocurred)
		return error_ocurred;

	this->trajectory = trajectory;
	return error_ocurred;
}

cvg_bool Controller_StateMachine_v2::reset() {
	current_state = MULTIROTOR_INIT_CONTROLMODE_SM;
	// the trajectory is replanified, to prevent from starting the controller with unplanified trajectory;
	return setTrajectory( trajectory, trajectory.traj_config );
}

void Controller_StateMachine_v2::initSM( cvg_double x_act, cvg_double y_act, cvg_double z_act, cvg_double yaw_act) {

	// Set feedforward outputs to default value
	vxfo = 0.0; vyfo = 0.0;  vzfo = 0.0;
	pitchfo = 0.0; rollfo = 0.0; dyawfo = 0.0;

	// Set reference values for the position controller
	xrefo = x_act;
	yrefo = y_act;
	zrefo = z_act;
	yawfo = yaw_act;

	// SM_Route current information
	checkpoint = trajectory.getInitialCheckpoint();
	true_checkpoint = checkpoint;
	h_stay_in_last_checkpoint = false;
	if (trajectory.getLength() == 1) { // the trajectory has only one checkpoint -> HOVER
		trajectory[checkpoint].convert2Vector(h_checkpoint);
		current_state    = SM_stateNames::HOVER;
		h_stay_in_last_checkpoint = true;
	} else {
		if (trajectory.getLength() == 0) {
			current_state    = SM_stateNames::POSITION_CONTROL;
		} else {
			current_state    = SM_stateNames::STRAIGHT;
		}
	}
	justChangedState = true;
}

// When the controller is in "controlMode"= POSITION_CONTROL or SPEED_CONTROL, then the Controller_StateMachine is bypassed.
// The state machine will not be executed and it would  be reseted before entering trajectory following control again.
// Instead, the commands are directly passed to the middle level controller (either position or speed commands)
void Controller_StateMachine_v2::processPositionControl() {
}

void Controller_StateMachine_v2::processSpeedControl() {
}

cvg_bool Controller_StateMachine_v2::activateTrajectoryControl() {
	cvg_bool error_ocurred = false;
	error_ocurred = reset();

	if (error_ocurred)
		return error_ocurred;

	derivBlock_vxfo.reset();
	derivBlock_vyfo.reset();

	// current_state cambia al ejecturase initSM()
	// started = false, causa la ejecucion de initSM();
	started = false;
	return error_ocurred;
}

// END: ***************************** General State Machine Functions ************************************



// ***************************** Straight line state: data and functions ****************************

void Controller_StateMachine_v2::processStraight() {

	// coordinates of line origin/initial point
	double x0 = r_p0.getValueData(1);
	double y0 = r_p0.getValueData(2);
	double z0 = r_p0.getValueData(3);

	// Vector indicating axis/line direction
	double ur_x = r_ur.getValueData(1);
	double ur_y = r_ur.getValueData(2);
	double ur_z = r_ur.getValueData(3);

	rs_act = ((xei-x0)*ur_x + (yei-y0)*ur_y + (zei-z0)*ur_z)/sqrt(ur_x*ur_x + ur_y*ur_y + ur_z*ur_z);

	xrefo = x0+rs_act*ur_x;
	yrefo = y0+rs_act*ur_y;
	zrefo = z0+rs_act*ur_z;

	double v_current = pow(vxei*vxei + vyei*vyei + vzei*vzei, 0.5);
	double vref = trajectory.obtainPlannedSpeed(checkpoint, xrefo, yrefo, zrefo, v_current);
	vxfo = vref*ur_x;
	vyfo = vref*ur_y;
	vzfo = vref*ur_z;

	derivBlock_vxfo.setInput( vxfo);
	dvxfo = derivBlock_vxfo.getOutput();
	derivBlock_vyfo.setInput( vyfo);
	dvyfo = derivBlock_vyfo.getOutput();
#ifdef SM_TRAJECTORYMODE_ACTIVATE_TILTFO
	double g = 9.81;
	pitchfo  = -asin( (cvg_double) dvxfo/g);
	rollfo   =  asin( (cvg_double) dvyfo/g);
	pitchfo *= 1/SM_TRAJECTORYMODE_TILTFO_RAD2TILTREF;
	rollfo  *= 1/SM_TRAJECTORYMODE_TILTFO_RAD2TILTREF;
#else
	pitchfo = 0.0;
	rollfo  = 0.0;
#endif
}

void Controller_StateMachine_v2::isStraightFinished() {

	if ( rs_act > rs_end) {  // straight phase finished
		pr_checkpoint = checkpoint; 					// previous checkpoint
		trajectory.achievedCheckpoint(checkpoint);
		checkpoint = trajectory.incrementCheckpoint(checkpoint);
		if ( r_nextState == SM_stateNames::HOVER ) {    // in this case the trajectory should be non-periodic and the end of the trajectory has been reached
			if ( !(trajectory.getIsPeriodic()) ) {		// Hover to trajectory end
				// h_checkpoint = trajectory[checkpoint];
				trajectory[checkpoint].convert2Vector(h_checkpoint);	// should be (trajectory.getLength()-1)
				current_state    = r_nextState; 		// HOVER
				h_stay_in_last_checkpoint = true;
				justChangedState = true;
				return;
			} else {
				// TODO: add trajEvent, a JP_TRAJ_ERROR, the impossible happened: reached end of periodic trajectory
			}
		} else {
			if ( r_nextState == SM_stateNames::TURN ) {	// perform turn
				current_state    = r_nextState; 		// TURN
				justChangedState = true;
				return;
			} else { // r_nextState == SM_stateNames::STRAIGHT
				if ( trajectory.isAchievedTrueCheckpoint( pr_checkpoint ) ) { // Proceed to next checkpoint
					current_state    = r_nextState; 	// STRAIGHT
					derivBlock_vxfo.reset();	// to avoid a tiltfi pick
					derivBlock_vyfo.reset();	// to avoid a tiltfi pick
					justChangedState = true;
					return;
				} else {  // hovert to pr_checkpoint
					// h_checkpoint     = trajectory[pr_checkpoint]
					trajectory[pr_checkpoint].convert2Vector(h_checkpoint);
					current_state    = SM_stateNames::HOVER; 	// HOVER
					h_stay_in_last_checkpoint = false;
					justChangedState = true;
					return;
				}
			}
		}
	} else { //	safety zone STRAIGHT mode
		cvg_double distance2straightLine = sqrt( pow( xrefo-xei,2) + pow( yrefo-yei,2) + pow( zrefo-zei,2) );
		if ( distance2straightLine > trajectory.traj_config.straighmode_safetyzone_radius_m ) {
//			hover to perpendicular point (assign to h_checkpoint) in the straight line
			Controller_SM_TrajectoryWaypoint new_waypoint( xrefo, yrefo, zrefo);
			new_waypoint.convert2Vector(h_checkpoint);
			current_state    = SM_stateNames::HOVER; 	// HOVER
			h_stay_in_last_checkpoint = false;
			justChangedState = true;
			return;
		} else {
			if ( rs_act < -trajectory.traj_config.straighmode_safetyzone_radius_m ) {
				// hover to r_p0
				h_checkpoint = r_p0;
				current_state    = SM_stateNames::HOVER; 	// HOVER
				h_stay_in_last_checkpoint = false;
				justChangedState = true;
				return;
			} else { // Continue STRAIGHT, nothing to be done here
			}
		}
	}
}

//cvg_double Controller_StateMachine_v2::calculateVmax(Vector &r_ur) {
//
//	double ur_x, ur_y, ur_z;
//	jesus_library::getVectorComponents( r_ur, ur_x, ur_y, ur_z);
//	double ur_xy = sqrt( ur_x*ur_x + ur_y*ur_y );
//
//	ur_z = fabs(ur_z);
//
//	double v1 = MULTIROTOR_TRAJECTORYCONTROLLER_VXY_AT_MAX/ur_xy;
//	double v2 = MULTIROTOR_TRAJECTORYCONTROLLER_VZ_AT_MAX/ur_z;
//
//	return (v1 < v2) ? v1 : v2;
//
//}

// END: ***************************** Straight line state: data and functions ****************************



// ***************************** Perform turn state: data and functions *****************************
//void Controller_StateMachine_v2::calculateTurn( Vector &r_ur, Vector &r_ur2, Vector &r_p1,
//		Vector &c_pinit, Vector &c_pc, Vector &c_pend, Vector &c_u0,
//		cvg_double &c_alim, cvg_double &c_vc, cvg_double &c_Rt, SM_stateNames::stateNames &nextState, double deltaL2) {
//
//}

//void Controller_StateMachine_v2::calculate_turn(cvg_int turn, Vector &c_pinit, Vector &c_pend, Vector &c_pc, cvg_double &c_alim, Vector &c_u0) {
//	trajectory.calculate_turn( turn, c_pinit, c_pend, c_pc, c_alim, c_u0);
//}

void Controller_StateMachine_v2::processTurn() {	// "during:"

	// 1º proyectar posicion sobre el plano r_ur, r_ur2
	Vector aux_ur2(3), aux_vector(3);
	aux_vector.copy(&r_ur);
	jesus_library::multiplyDoubleVsVector( jesus_library::dotProduct(r_ur, r_ur2), aux_vector );
	aux_ur2.substraction( &r_ur2, &aux_vector);
	jesus_library::unitarizeVector(aux_ur2); // vector ortogonal a r_ur; para formar base ortonormal en el plano r_ur, r_ur2

	Vector pos_act(3);
	pos_act.setValueData(xei,1);
	pos_act.setValueData(yei,2);
	pos_act.setValueData(zei,3);

	aux_vector.substraction( &pos_act, &c_pc);
	Vector pos_act_proy_ur(3),pos_act_proy_ur2(3);
	pos_act_proy_ur.copy(&r_ur);
	pos_act_proy_ur2.copy(&aux_ur2);

	jesus_library::multiplyDoubleVsVector( jesus_library::dotProduct(r_ur, aux_vector), pos_act_proy_ur );
	jesus_library::multiplyDoubleVsVector( jesus_library::dotProduct(aux_ur2, aux_vector), pos_act_proy_ur2 );

	Vector pos_act_proy(3);
	pos_act_proy.addition( &pos_act_proy_ur, &pos_act_proy_ur2);
	cvg_double current_radius = jesus_library::normOfVector(pos_act_proy);

	#ifdef SM_STATEMACHINE_DEBUG
	std::cout << "c_pc = \n"; c_pc.mostrar();
	std::cout << "c_pinit = \n"; c_pinit.mostrar();
	std::cout << "c_pend = \n"; c_pend.mostrar();
	std::cout << "pos_act_proy = \n"; pos_act_proy.mostrar();
	#endif // SM_STATEMACHINE_DEBUG

	// 2º localizar pref
	Vector u_ro(3);
	u_ro.copy(&pos_act_proy);
	jesus_library::unitarizeVector(u_ro);
	aux_vector.copy(&u_ro);
	jesus_library::multiplyDoubleVsVector( c_Rt, aux_vector);
	Vector pos_ref(3);
	pos_ref.addition(&c_pc, &aux_vector);
	jesus_library::getVectorComponents(pos_ref, xrefo, yrefo, zrefo);
	cvg_double current_altitude_error = (zei-zrefo); // fabs(zei-zrefo);

	// 3º calcular velocidad de referencia
	Vector u_fi(3);
	jesus_library::crossProduct(u_fi, c_u0, u_ro);
	jesus_library::unitarizeVector(u_fi);
	Vector v_ref(3);
	v_ref.copy(&u_fi);
	jesus_library::multiplyDoubleVsVector(c_vc, v_ref);
	jesus_library::getVectorComponents(v_ref, vxfo, vyfo, vzfo);


//	// 4º Calcular pitcho, rollo
	derivBlock_vxfo.setInput( vxfo);
	dvxfo = derivBlock_vxfo.getOutput();
	derivBlock_vyfo.setInput( vyfo);
	dvyfo = derivBlock_vyfo.getOutput();
#ifdef SM_TRAJECTORYMODE_ACTIVATE_TILTFO
	double g = 9.81;
	pitchfo  = -asin( (cvg_double) dvxfo/g);
	rollfo   =  asin( (cvg_double) dvyfo/g);
	pitchfo *= 1/SM_TRAJECTORYMODE_TILTFO_RAD2TILTREF;
	rollfo  *= 1/SM_TRAJECTORYMODE_TILTFO_RAD2TILTREF;
#else
	pitchfo = 0.0;
	rollfo  = 0.0;
#endif
//  // This is a code that never work that was intended to stop the parrot in it's current position
//	double v_act = sqrt( vxei*vxei + vyei*vyei + vzei*vzei );
//	double act_req  = pow(v_act,2)/c_Rt;
//	double tilt_req = act_req/SM_STATEMACHINE_G;
//	aux_vector.copy(&u_ro);
//	jesus_library::multiplyDoubleVsVector( -tilt_req, aux_vector);
//	aux_vector.setValueData( 0.0, 3);
//
//	Vector parrot_ux(3), parrot_uy(3);
//	parrot_ux.setValueData( cos(yawei), 1);
//	parrot_ux.setValueData( sin(yawei), 2);
//	parrot_uy.setValueData(-sin(yawei), 1);
//	parrot_uy.setValueData( cos(yawei), 2);
//
//	pitchfo = -jesus_library::dotProduct( parrot_ux, aux_vector)*SM_STATEMACHINE_TILTCOMM_CORRECT_FACTOR;
//	rollfo  =  jesus_library::dotProduct( parrot_uy, aux_vector)*SM_STATEMACHINE_TILTCOMM_CORRECT_FACTOR;
//	pitchfo /= SM_STATEMACHINE_TILTCOMM_NORMALIZATION_CONSTANT;
//	rollfo  /= SM_STATEMACHINE_TILTCOMM_NORMALIZATION_CONSTANT;

	// 5º Calculo de alpha y comparacion con c_alim
	Vector aux_vector2(3);
	aux_vector.substraction(&c_pinit,&c_pc);
	jesus_library::unitarizeVector(aux_vector);
	cvg_double aux = jesus_library::dotProduct(u_ro, aux_vector);
	jesus_library::saturate( aux, -1, 1);
	jesus_library::crossProduct(aux_vector2, aux_vector, u_ro);
	cvg_double angle_sign = ( jesus_library::dotProduct(c_u0, aux_vector2) > 0.0 ) ? +1.0 : -1.0;
	cvg_double current_alpha = angle_sign*acos(aux);

	#ifdef SM_STATEMACHINE_DEBUG
	std::cout << "c_alim = " << c_alim << "; alpha = " << current_alpha << "\n";
	#endif // SM_STATEMACHINE_DEBUG

	if ( current_alpha > c_alim) {  // ended the turn
		c_nextState = SM_stateNames::STRAIGHT;
		c_changeState = true;
		return;
	} else {  // turn not ended, check safety zones using: current_radius, current_alpha
		if ( ( current_alpha < trajectory.traj_config.turnmode_safetyzone_negalpha_rad ) ||
				( fabs(current_radius - c_Rt) > trajectory.traj_config.turnmode_safetyzone_radius_m ) ||
				( fabs(current_altitude_error) > trajectory.traj_config.turnmode_safetyzone_altitude_m ) ) { // enter hover to prev checkpoint
#ifdef SM_STATEMACHINE_DEBUG
			std::cout << "pos_act_proy_ur < 0\n";
#endif // SM_STATEMACHINE_DEBUG
			trajectory[pr_checkpoint].convert2Vector(h_checkpoint);
			c_nextState = SM_stateNames::HOVER;
			h_stay_in_last_checkpoint = false;
			c_changeState = true;

			// In this case I have to redefine the state machine output
			jesus_library::getVectorComponents(h_checkpoint, xrefo, yrefo, zrefo);
			vxfo = 0.0;
			vyfo = 0.0;
			vzfo = 0.0;
			pitchfo = 0.0;
			rollfo  = 0.0;

			return;
		} else { // Continue turn, nothing else to do
			return;
		}
	}

}

void Controller_StateMachine_v2::isTurnFinished() { 	// Decision making is implemented in point (5º) of processTurn()
	if (c_changeState) {
		current_state = c_nextState;
		justChangedState = true;
		c_changeState = false;
	}
}

// END: ***************************** Perform turn state: data and functions *****************************



// ***************************** Hover to checkpoint state: data and functions **********************
void Controller_StateMachine_v2::processHover() {		// "during:"
	// 1º) Calculate mid-level controller references
	// Hover mode goes to h_checkpoint
	jesus_library::getVectorComponents( h_checkpoint, xrefo, yrefo, zrefo);

	vxfo = 0.0;
	vyfo = 0.0;
	vzfo = 0.0;
	pitchfo = 0.0;
	rollfo  = 0.0;

	// 2º) isHoverFinished()
	cvg_double dist2h_checkpoint = sqrt( pow( (xei - xrefo), 2) + pow( (yei - yrefo), 2) + pow( (zei - zrefo), 2) );
	if ( dist2h_checkpoint < trajectory.traj_config.chk_clearance_R ) {
		if ( trajectory.routeFinished(checkpoint) ) {
			if ( !(h_stay_in_last_checkpoint) ) { 		// this condition is probably never going to be reached
				trajectory[checkpoint].convert2Vector(h_checkpoint);
				h_stay_in_last_checkpoint = true;
				current_state = SM_stateNames::HOVER;
				justChangedState = true;
				return;
			}
			return;
		} else {
			if ( trajectory.getLength() > 1 ) {
				current_state = SM_stateNames::STRAIGHT;
				derivBlock_vxfo.reset();
				derivBlock_vyfo.reset();
				justChangedState = true;
			} else {
				if ( !(h_stay_in_last_checkpoint) ) {
					trajectory[checkpoint].convert2Vector(h_checkpoint);
					h_stay_in_last_checkpoint = true;
					current_state = SM_stateNames::HOVER;
					justChangedState = true;
					return;
				}
				return;
			}
		}
	} else { // continue hovering to h_checkpoint
		return;
	}

}

void Controller_StateMachine_v2::isHoverFinished() {		// transition
	// Decision making is implemented in point (2º) of processHover()
}

// END: ***************************** Hover to checkpoint state: data and functions **********************
