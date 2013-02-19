///*
// * task_scheduler.cpp
// *
// *  Created on: Jun 29, 2012
// *      Author: jespestana
// */
//
//#include "controller/task_scheduler.h"
//#include "MyDrone.h"
//
//#define PLANIFIER_SLEEP_SEC 0.05
//#define PLANIFIER_WAIT_TIMEOUT_AFTER_TAKEOFF   10.0
//#define PLANIFIER_WAIT_TIME_FRONT_OF_PYLONS    2.0
//#define PLANIFIER_X_POSITION_FRONT_OF_PYLONS1  3.0
//#define PLANIFIER_X_POSITION_FRONT_OF_PYLONS2 -3.0
//#define PLANIFIER_PYLONS_SEARCH_WAIT_TIME      1.0
//#define PLANIFIER_PYLONS_SEARCH_DISTANCE_STEPS 1.5
//
//
//Task_scheduler::Task_scheduler(PelicanEKF *pelicanEKF) : 	planifier_started(false),
//																		reset_flag(false),
//																		cvgThread("Planifier_Pylon_Challenge_Thread"),
//																		estObserv(PARROTMODEL_NUMMEASURES) {
//
//	p2pelicanEKF = pelicanEKF;
//	start();
//	debugString = "planifier not started";
//	desired_yaw = 0.0;
//}
//
//Task_scheduler::~Task_scheduler() {
//	stop();
//}
//
//void Task_scheduler::setMyDrone(MyDrone *drone) {
//	if ( planifierMutex.lock() ) {
//		myDrone = drone;
//		planifierMutex.unlock();
//	}
//}
//
//	void Task_scheduler::reset() {
//		reset_flag = true;
//		planifier_started = false;
//	}
//
//	void Task_scheduler::takeoffTask() {
//
//		debugString = "take off task";
//
//		// Take-off and startEKF
////		case 't':
//		myDrone->setControlMode(TAKEOFF);
////      case 'k':		// start kalman filter
//		myDrone->startEKF();
//
//		// When the Pelican starts hovering:
//		// 	1) move
//		//  2) start position controller in actual position
//		//  3) set altitude to one meter
////		FlyingMode act_flyingMode; // = myDrone->getControlMode();
//		char act_drone_state = myDrone->getDroneState();
//		bool subtask_finished = false;
//		while (!subtask_finished) {
////			act_flyingMode = myDrone->getControlMode();
//			act_drone_state = myDrone->getDroneState();
////			if ( (act_flyingMode == HOVER) || (act_flyingMode == MOVE) ) {
//			if ( (act_drone_state == HOVERING) || (act_drone_state == FLYING) ) {
//				subtask_finished = true;
//
////				case 'm': set Flying mode to MOVE
////				case 'j':		// start controller
////				case 'g':		// start position controller
//				myDrone->setControlMode(MOVE);
//				if ( !myDrone->isEKFStarted() ) {
//					myDrone->startEKF();
//				}
//				myDrone->startController();
//				myDrone->setPositionControl();
////				case 's':
//				myDrone->setControllerAllReferences2ActualValues();
//				myDrone->setControllerZReference( 1.3f );
//			}
//			whileLoopMandatoryCode();
//		}
//		subtask_finished = false;
//
//		while (!subtask_finished) {
//			p2pelicanEKF->getEstimatedQuadrotorPosition( x_ekf, y_ekf, z_ekf, yaw_ekf);
//			if ( ( z_ekf > 1.20 ) ) {
//				subtask_finished = true;
//			}
//			whileLoopMandatoryCode();
//		}
//
//	}
//
//	void Task_scheduler::finalLandingTask() {
//		myDrone->setPositionControl();
//		myDrone->setControllerAllReferences2ActualValues();
//		myDrone->setControllerZReference( 0.0f );
//
//
//		p2pelicanEKF->getEstimatedQuadrotorPosition( x_ekf, y_ekf, z_ekf, yaw_ekf);
//		// wait 30 seconds and land
//		while ( z_ekf > 0.30 ) {
//			p2pelicanEKF->getEstimatedQuadrotorPosition( x_ekf, y_ekf, z_ekf, yaw_ekf);
//			whileLoopMandatoryCode();
//		}
//
//		myDrone->setControlMode(LAND);
//	}
//
//	void Task_scheduler::searchPylons1() {
//
//		debugString = "search Pylons1";
//
//		// Vuelvo a activar el control de posicion
//		myDrone->setPositionControl();
//		myDrone->setControllerAllReferences2ActualValues();
//		yawTurnToZero();
//		myDrone->setControllerZReference( 1.2 );
//
//		bool subtask_finished = false;
//		cvg_double xci_out, yci_out, yawci_out, zci_out;
//		while (!subtask_finished) {
//			myDrone->getCntReferences( &xci_out, &yci_out, &yawci_out, &zci_out);
//			p2pelicanEKF->getEstimatedQuadrotorPosition( x_ekf, y_ekf, z_ekf, yaw_ekf);
//			if ( fabs( xci_out - x_ekf ) < 0.4 ) {
//				myDrone->setControllerXReference( xci_out+PLANIFIER_PYLONS_SEARCH_DISTANCE_STEPS );
//				waitSeconds(PLANIFIER_PYLONS_SEARCH_WAIT_TIME);
//			}
//
//			if ( myDrone->localized_poles_1 ) {
//				subtask_finished = true;
//				myDrone->setControllerXReference(PLANIFIER_X_POSITION_FRONT_OF_PYLONS1);
//				myDrone->setControllerYReference(+0.0);
//			}
//
//			whileLoopMandatoryCode();
//		}
//		subtask_finished = false;
//
//		while (!subtask_finished) {
//			p2pelicanEKF->getEstimatedQuadrotorPosition( x_ekf, y_ekf, z_ekf, yaw_ekf);
//			myDrone->setControllerXReference(PLANIFIER_X_POSITION_FRONT_OF_PYLONS1);
//			myDrone->setControllerYReference(+0.0);
//			if ( ( fabs(x_ekf - PLANIFIER_X_POSITION_FRONT_OF_PYLONS1) < 0.4 ) && ( fabs(y_ekf - 0.0) < 0.40 ) ) {
//				subtask_finished = true;
//			}
//			whileLoopMandatoryCode();
//		}
//
//		waitSeconds( PLANIFIER_WAIT_TIME_FRONT_OF_PYLONS );
//
//	}
//
//	void Task_scheduler::makeHalfEightPylons1() {
//
//		debugString = "makeHalfEight1";
//
//		myDrone->setEightTrajectory();
//		yawTurnToZero();
//
//		bool subtask_finished = false;
//		while (!subtask_finished) {
//			p2pelicanEKF->getEstimatedQuadrotorPosition( x_ekf, y_ekf, z_ekf, yaw_ekf);
//
//			if ( ( ( myDrone->getCheckpoint() == 7 ) || ( myDrone->getTrueCheckpoint() == 7 ) ) && ( z_ekf < 1.5 ) ) { // && ( x_ekf < 2.5 )
//				subtask_finished = true;
//			}
//			whileLoopMandatoryCode();
//		}
//		subtask_finished = false;
//	}
//
//
//	inline void Task_scheduler::whileLoopMandatoryCode() {
//		usleep( PLANIFIER_SLEEP_SEC*1e6 );
//		myDrone->setControllerYawReference( desired_yaw );
//
//		// If the quadrotor starts hovering -> Start controller & and set Yaw
//		char act_drone_state = myDrone->getDroneState();
//		if ( (act_drone_state == HOVERING) ) {
//			myDrone->setControlMode(MOVE);
//			myDrone->startController();
//			myDrone->setControllerYawReference( desired_yaw );
//		}
//
////		if (reset_flag) {
////			finalLandingTask();
////			reset_flag = false;
////			return;
////		}
//	}
//
////	void Task_scheduler::landingDueToResetTask() {
////
////	}
//
//	void Task_scheduler::yawTurnToZero() {
//		desired_yaw = 0.0;
//		myDrone->setControllerYawReference( 0.0 );
//	}
//
//	void Task_scheduler::searchPylons2() {
//
//		debugString = "search Pylons2";
//
//		// Vuelvo a activar el control de posicion
//		myDrone->setPositionControl();
//		myDrone->setControllerAllReferences2ActualValues();
//		yawTurnToMinusPI();
//		myDrone->setControllerZReference( 1.2 );
//
//		bool subtask_finished = false;
//		cvg_double xci_out, yci_out, yawci_out, zci_out;
//		while (!subtask_finished) {
//			myDrone->getCntReferences( &xci_out, &yci_out, &yawci_out, &zci_out);
//			p2pelicanEKF->getEstimatedQuadrotorPosition( x_ekf, y_ekf, z_ekf, yaw_ekf);
//			if ( fabs( xci_out - x_ekf ) < 0.4 ) {
//				debugString = "search Pylons2: move forward";
//				myDrone->setControllerXReference( xci_out-PLANIFIER_PYLONS_SEARCH_DISTANCE_STEPS );
//				waitSeconds(PLANIFIER_PYLONS_SEARCH_WAIT_TIME);
//			}
//
//			if ( myDrone->localized_poles_2 ) {
//				debugString = "search Pylons2: going to [-2, 0]";
//				subtask_finished = true;
//				myDrone->setControllerXReference(PLANIFIER_X_POSITION_FRONT_OF_PYLONS2);
//				myDrone->setControllerYReference(+0.0);
//			}
//
//			whileLoopMandatoryCode();
//		}
//		subtask_finished = false;
//
//		while (!subtask_finished) {
//			p2pelicanEKF->getEstimatedQuadrotorPosition( x_ekf, y_ekf, z_ekf, yaw_ekf);
//			debugString = "search Pylons2: waiting to [-2 0]";
//			myDrone->setControllerXReference(PLANIFIER_X_POSITION_FRONT_OF_PYLONS2);
//			myDrone->setControllerYReference(+0.0);
//			if ( ( fabs(x_ekf - (PLANIFIER_X_POSITION_FRONT_OF_PYLONS2)) < 0.4 ) && ( fabs(y_ekf - 0.0) < 0.40 ) ) {
//				debugString = "search Pylons2: arrived to [-2 0]";
//				subtask_finished = true;
//			}
//			whileLoopMandatoryCode();
//		}
//
//		debugString = "search Pylons2: waiting 2 seconds";
//		waitSeconds(PLANIFIER_WAIT_TIME_FRONT_OF_PYLONS);
//
//	}
//
//	void Task_scheduler::yawTurnToMinusPI() {
//		desired_yaw = +M_PI;
//		myDrone->setControllerYawReference( +M_PI );
//	}
//
//	void Task_scheduler::makeHalfEightPylons2() {
//
//		debugString = "makeHalfEightPylons2";
//
//		myDrone->setEightTrajectory_2();
//		yawTurnToMinusPI();
//
//		bool subtask_finished = false;
//		while (!subtask_finished) {
//			p2pelicanEKF->getEstimatedQuadrotorPosition( x_ekf, y_ekf, z_ekf, yaw_ekf);
//			if ( ( ( myDrone->getCheckpoint() == 2 ) || ( myDrone->getTrueCheckpoint() == 2 ) )  && ( fabs(z_ekf) < 1.5 ) ) { // && ( fabs(x_ekf) > -2.5 )
//				subtask_finished = true;
//			}
//			whileLoopMandatoryCode();
//		}
//		subtask_finished = false;
//	}
//
//	void Task_scheduler::run()   {
//
//		float elapsed = 0.0;
//
//		if (!planifier_started) {
//			usleep( PLANIFIER_SLEEP_SEC*1e6/3 );
//			return;
//		}
//
//		takeoffTask();
//
//		// wait 10 seconds after take-off
//		timer.restart(false);
//		elapsed = timer.getElapsedSeconds();
//		while ( elapsed < PLANIFIER_WAIT_TIMEOUT_AFTER_TAKEOFF ) {
//			yawTurnToZero();
//			whileLoopMandatoryCode();
//			elapsed = timer.getElapsedSeconds();
//		}
//
//		// loop 1
//		yawTurnToZero();
//		searchPylons1();
//
//		yawTurnToZero();
//		makeHalfEightPylons1();
//
//		searchPylons2();
//		makeHalfEightPylons2();
//
//		// loop 2
//		yawTurnToZero();
//		searchPylons1();
//
//		yawTurnToZero();
//		makeHalfEightPylons1();
//
//		yawTurnToMinusPI();
//		searchPylons2();
//		makeHalfEightPylons2();
//
//		// loop 3
//		yawTurnToZero();
//		searchPylons1();
//
//		yawTurnToZero();
//		makeHalfEightPylons1();
//
//		yawTurnToMinusPI();
//		searchPylons2();
//		makeHalfEightPylons2();
//
//		// loop 4
//		yawTurnToZero();
//		searchPylons1();
//
//		yawTurnToZero();
//		makeHalfEightPylons1();
//
//		searchPylons2();
//		makeHalfEightPylons2();
//
//		// loop 5
//		yawTurnToZero();
//		searchPylons1();
//
//		yawTurnToZero();
//		makeHalfEightPylons1();
//
//		yawTurnToMinusPI();
//		searchPylons2();
//		makeHalfEightPylons2();
//
//		// loop 6
//		yawTurnToZero();
//		searchPylons1();
//
//		yawTurnToZero();
//		makeHalfEightPylons1();
//
//		yawTurnToMinusPI();
//		searchPylons2();
//		makeHalfEightPylons2();
//
//
//		debugString = "Landing after 3 eight laps";
//
//		// Stay in position and land
//		yawTurnToZero();
//		myDrone->setPositionControl();
//		myDrone->setControllerAllReferences2ActualValues();
//		myDrone->setControllerZReference( 1.2 );
//
//		// wait 10 seconds and land
//		timer.restart(false);
//		elapsed = timer.getElapsedSeconds();
//		while ( elapsed < 10.0 ) {
//			whileLoopMandatoryCode();
//			elapsed = timer.getElapsedSeconds();
//		}
//
//		finalLandingTask();
//		planifier_started = false;
//	}
//
//
