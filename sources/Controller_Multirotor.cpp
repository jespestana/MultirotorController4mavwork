/*
 * Controller_Multirotor.cpp
 *
 *  Created on: Nov 20, 2012
 *      Author: jespestana
 */

#include "controller/Controller_Multirotor.h"
#include "MyDrone.h"
#include <unistd.h>		// usleep function

#ifdef _MULTIROTOR_IS_PARROT_
#include "controller/config/parrot/config_controller_Parrot.h"
#endif
#ifdef _MULTIROTOR_IS_PELICAN_
#include "controller/config/pelican/config_controller_Pelican.h"
#endif


Controller_Multirotor::Controller_Multirotor() : 	controller_started(false),
													cvgThread("Speed Controller Thread"),
													estObserv(MULTIROTOR_MODEL_NUMMEASURES),
													midlevel_controller() {
	Ts = MULTIROTOR_SPEEDCONTROLLER_TS;
	start();

	control_mode = MULTIROTOR_INIT_CONTROLMODE;

	pitch_lowpassfilter.setResponseTime(MULTIROTOR_TILT_REFERENCE_CUTOFF_TR);
	pitch_lowpassfilter.enableSaturation( true, -MULTIROTOR_SPEEDCONTROLLER_MAX_PITCH, +MULTIROTOR_SPEEDCONTROLLER_MAX_PITCH);
	pitch_lowpassfilter.reset();
	roll_lowpassfilter.setResponseTime(MULTIROTOR_TILT_REFERENCE_CUTOFF_TR);
	roll_lowpassfilter.enableSaturation( true, -MULTIROTOR_SPEEDCONTROLLER_MAX_ROLL, +MULTIROTOR_SPEEDCONTROLLER_MAX_ROLL);
	roll_lowpassfilter.reset();
	dyaw_lowpassfilter.setResponseTime(MULTIROTOR_DYAW_REFERENCE_CUTOFF_TR);
	dyaw_lowpassfilter.enableSaturation( true, -MULTIROTOR_SPEEDCONTROLLER_DYAWMAX, +MULTIROTOR_SPEEDCONTROLLER_DYAWMAX);
	dyaw_lowpassfilter.reset();
	dz_lowpassfilter.setResponseTime(MULTIROTOR_DALT_REFERENCE_CUTOFF_TR);
	dz_lowpassfilter.enableSaturation( true, -MULTIROTOR_SPEEDCONTROLLER_DZMAX, +MULTIROTOR_SPEEDCONTROLLER_DZMAX);
	dz_lowpassfilter.reset();
}

void Controller_Multirotor::setMyDrone(MyDrone *p2my_drone) {
	if ( controllerMutex.lock() ) {
		pmy_drone = p2my_drone;
		controllerMutex.unlock();
	}
}

Controller_Multirotor::~Controller_Multirotor() {
	stop();
}

void Controller_Multirotor::startController() {
	pmy_drone->startStateObsever();
	if (!controller_started) {
		controller_started = true;
		reset();
	}
}

void Controller_Multirotor::stopController() {
	pmy_drone->setControlData( 0.0, 0.0, 0.0, 0.0);
	controller_started = false;
	reset();
}

void Controller_Multirotor::reset() {

	if ( controllerMutex.lock() ) {
		control_mode = MULTIROTOR_INIT_CONTROLMODE;

		vxfi = 0.0; vyfi = 0.0; pitchfi = 0.0; rollfi = 0.0;		// fi ~ feedforward inputs
		// Next control layer commands, in this case these commands are to be sent directly to the drone using the pelican proxy
		vxco_int = 0.0; vyco_int = 0.0; yawco_int = 0.0; zco_int = 0.0;	// co_int ~ command outputs internal (to speed controller)
		pitchco = 0.0; rollco = 0.0; dyawco = 0.0; dzco = 0.0;

		midlevel_controller.reset();
		state_machine.reset();

		pitch_lowpassfilter.reset();
		roll_lowpassfilter.reset();
		dyaw_lowpassfilter.reset();
		dz_lowpassfilter.reset();

		timer.restart(false);
		controllerMutex.unlock();
	}

}

void Controller_Multirotor::run() {

	if (!controller_started) {
		usleep( MULTIROTOR_SPEEDCONTROLLER_TS*1e6/3 );
		return;
	}

	cvg_double elapsed = 0.0;
	if ( controllerMutex.lock() ) {
		elapsed = timer.getElapsedSeconds();
		controllerMutex.unlock();
	} else { return; }

	if ( elapsed >= MULTIROTOR_SPEEDCONTROLLER_TS ) {
		// Obtener medidas del EKF
		(*pmy_drone).state_observer.getEstimatedObservation(&estObserv);
		if ( controllerMutex.lock() ) {
			yaws = estObserv.getValueData( 4);
			dyaws = estObserv.getValueData(3);
			zs   = estObserv.getValueData( 6);
			xs  = estObserv.getValueData( 7);
			ys  = estObserv.getValueData( 8);
			vzs = estObserv.getValueData( 5);
			vxs  = estObserv.getValueData( 9);
			vys  = estObserv.getValueData(10);
			controllerMutex.unlock();
		}
	} else {
		usleep( MULTIROTOR_SPEEDCONTROLLER_TS*1e6/3 );
		return;
	}

	if ( controllerMutex.lock() ) {
		// Obtener valor de los comandos de referencia
		// Los comandos de referencia se encuentran directamente en: vxci, vyci, yawci, zci;

		// Estimaciones del EKF ya obtenidas y guardadas en el vector *estObserv; hecho en el if anterior.

		// Corregir uno de los valores de yaw para controlar correctamente
		yaws = jesus_library::mapAnglesToBeNear_PIrads( yaws, yawci);

		// Calculos del controlador
		switch (control_mode) {
		case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
			state_machine.setInputs( xs, ys, zs, vxs, vys, vzs, yaws, dyaws);
//			state_machine.setInputs( xs, ys, zci, vxs, vys, vzs, yaws, dyaws);
			double xci2, yci2, zci2, vxfi2, vyfi2, dzfi2, yawci2, dyawfi2, pitchfi2, rollfi2;
			state_machine.getOutput( xci2, yci2, zci2, vxfi2, vyfi2, dzfi2, yawci2, dyawfi2, pitchfi2, rollfi2);

			// NOTE: the state_machine does not do anything with the yaw, that why it is introduced from the outside
#ifdef SM_TRAJECTORYMODE_ACTIVATE_TILTFO
			setAllReferences( xci2, yci2, yawci, zci2, vxfi2, vyfi2, 0.0, dzfi2, pitchfi2, rollfi2);
#else
			setAllReferences( xci2, yci2, yawci, zci2, vxfi2, vyfi2, 0.0, dzfi2, 0.0, 0.0);
#endif
			midlevel_controller.setFeedback( xs, ys, vxs, vys, yaws, zs);
			midlevel_controller.setReference( xci, yci, yawci, zci, vxfi, vyfi, dyawfi, dzfi, pitchfi, rollfi);
			midlevel_controller.getOutput( &pitchco_hf, &rollco_hf, &dyawco_hf, &dzco_hf);
			break;
		case Controller_MidLevel_controlMode::SPEED_CONTROL:
			setAllReferences( xs, ys, yawci, zci, vxfi, vyfi);
			midlevel_controller.setFeedback( xs, ys, vxs, vys, yaws, zs);
			midlevel_controller.setReference( xci, yci, yawci, zci, vxfi, vyfi);
			midlevel_controller.getOutput( &pitchco_hf, &rollco_hf, &dyawco_hf, &dzco_hf);
			break;
		case Controller_MidLevel_controlMode::POSITION_CONTROL:
			midlevel_controller.setFeedback( xs, ys, vxs, vys, yaws, zs);
			midlevel_controller.setReference( xci, yci, yawci, zci);
			midlevel_controller.getOutput( &pitchco_hf, &rollco_hf, &dyawco_hf, &dzco_hf);
			break;
		}

		// (low pass) filter the command outputs
		pitch_lowpassfilter.setInput(pitchco_hf);
		pitchco = pitch_lowpassfilter.getOutput();
		roll_lowpassfilter.setInput(rollco_hf);
		rollco = roll_lowpassfilter.getOutput();
		dyaw_lowpassfilter.setInput(dyawco_hf);
		dyawco = dyaw_lowpassfilter.getOutput();
		dz_lowpassfilter.setInput(dzco_hf);
		dzco = dz_lowpassfilter.getOutput();

		timer.restart( true );
		controllerMutex.unlock();
	}

	// Send commands to the multirotor
	pmy_drone->setControlData( rollco, pitchco, dzco, dyawco );

//	pmy_drone->setControlData( rollco, pitchco, 0.0, 0.0 );
//	pmy_drone->setControlData( rollco, pitchco, 0.0, dyawco );
//	pmy_drone->setControlData(DroneProxy::Comm::ControlChannel::THETA, 0.0f, pitchco, 0.0f, 0.0f);		// Pitch
//	pmy_drone->setControlData(DroneProxy::Comm::ControlChannel::PHI,  rollco, 0.0f, 0.0f,   0.0f);		// Roll
//	pmy_drone->setControlData(DroneProxy::Comm::ControlChannel::YAW,   0.0f, 0.0f, 0.0f,    dyawco);	// Yaw

	saveCntRefData2Disk();
	usleep( MULTIROTOR_SPEEDCONTROLLER_TS*1e6/3 );
}

void Controller_Multirotor::setAllReferences2ActualValues() {

	(*pmy_drone).state_observer.getEstimatedObservation(&estObserv);
	if ( controllerMutex.lock() ) {
		// Obtener medidas del EKF
		yaws = estObserv.getValueData( 4);
		zs   = estObserv.getValueData( 6);
		xs   = estObserv.getValueData( 7);
		ys   = estObserv.getValueData( 8);
//		vxs  = estObserv.getValueData( 9);
//		vys  = estObserv.getValueData(10);
		controllerMutex.unlock();
	}
	setAllReferences( xs, ys, yaws, zs, 0.0, 0.0, 0.0, 0.0);

}

// ***** Set controller reference functions *****
void Controller_Multirotor::setAllReferences( cvg_double xci_in, cvg_double yci_in, cvg_double yawci_in, cvg_double zci_in,
		cvg_double vxfi_t , cvg_double vyfi_t , cvg_double dyawfi_t , cvg_double dzfi_t ,
		cvg_double pitchfi_t , cvg_double rollfi_t ) {

	saveCntRefData2Disk();

	if ( controllerMutex.lock() ) {
		xci = xci_in;
		yci = yci_in;
		yawci_in = atan2( sin(yawci_in), cos(yawci_in));
		yawci= yawci_in;
		zci  = zci_in;

		vxfi = vxfi_t;
		vyfi = vyfi_t;
		pitchfi = pitchfi_t;
		rollfi = rollfi_t;
		dyawfi = dyawfi_t;
		dzfi   = dzfi_t;

		saveCntRefData2Disk();
		controllerMutex.unlock();
	}
}

void Controller_Multirotor::setXReference( cvg_double xci_in) {

	saveCntRefData2Disk();

	if ( controllerMutex.lock() ) {
		xci = xci_in;

		saveCntRefData2Disk();
		controllerMutex.unlock();
	}
}

void Controller_Multirotor::setYReference( cvg_double yci_in) {

	saveCntRefData2Disk();

	if ( controllerMutex.lock() ) {
		yci = yci_in;

		saveCntRefData2Disk();
		controllerMutex.unlock();
	}
}

void Controller_Multirotor::setYawReference( cvg_double yawci_in) {

	saveCntRefData2Disk();

	if ( controllerMutex.lock() ) {
		yawci_in = atan2( sin(yawci_in), cos(yawci_in));
		yawci= yawci_in;

		saveCntRefData2Disk();
		controllerMutex.unlock();
	}
}


void Controller_Multirotor::setZReference( cvg_double zci_in) {
	saveCntRefData2Disk();

	if ( controllerMutex.lock() ) {
		zci = zci_in;

		saveCntRefData2Disk();
		controllerMutex.unlock();
	}
}

void Controller_Multirotor::setVFIReference( cvg_double vxfi_in, cvg_double vyfi_in) {
	saveCntRefData2Disk();

	if ( controllerMutex.lock() ) {
		vxfi = vxfi_in;
		vyfi = vyfi_in;

		saveCntRefData2Disk();
		controllerMutex.unlock();
	}
}

void Controller_Multirotor::setVXFIReference( cvg_double vxfi_in) {
	saveCntRefData2Disk();

	if ( controllerMutex.lock() ) {
		vxfi = vxfi_in;

		saveCntRefData2Disk();
		controllerMutex.unlock();
	}
}

void Controller_Multirotor::setVYFIReference( cvg_double vyfi_in) {
	saveCntRefData2Disk();

	if ( controllerMutex.lock() ) {
		vyfi = vyfi_in;

		saveCntRefData2Disk();
		controllerMutex.unlock();
	}
}

void Controller_Multirotor::setPitchRollReference( cvg_double pitchfi_in, cvg_double rollfi_in) {
	saveCntRefData2Disk();

	if ( controllerMutex.lock() ) {
		pitchfi = pitchfi_in;
		rollfi  =  rollfi_in;

		saveCntRefData2Disk();
		controllerMutex.unlock();
	}
}

void Controller_Multirotor::setDYawFIReference( cvg_double dyawfi_in) {
	saveCntRefData2Disk();

	if ( controllerMutex.lock() ) {
		dyawfi = dyawfi_in;

		saveCntRefData2Disk();
		controllerMutex.unlock();
	}
}

void Controller_Multirotor::setDZFIReference( cvg_double dzfi_in) {
	saveCntRefData2Disk();

	if ( controllerMutex.lock() ) {
		dzfi = dzfi_in;

		saveCntRefData2Disk();
		controllerMutex.unlock();
	}
}
// END: ***** Set Controller functions *****

// ***** Get/Accessors Controller functions *****
void Controller_Multirotor::getLastOutput( cvg_double *pitchco_out, cvg_double *rollco_out, cvg_double *dyawco_out, cvg_double *dzco_out) {
	if ( controllerMutex.lock() ) {
		*pitchco_out= pitchco;
		*rollco_out = rollco;
		*dyawco_out = dyawco;
		*dzco_out   = dzco;
		controllerMutex.unlock();
	}
}

void Controller_Multirotor::getCntReferences( cvg_double *xci_out, cvg_double *yci_out, cvg_double *yawci_out, cvg_double *zci_out) {
	if ( controllerMutex.lock() ) {
		*xci_out   = xci;
		*yci_out   = yci;
		*yawci_out = yawci;
		*zci_out   = zci;
		controllerMutex.unlock();
	}
}

// ***** Scheduler part of MyDrone *****
SM_stateNames::stateNames Controller_Multirotor::getCurrentState() {
	SM_stateNames::stateNames act_state;
	if ( controllerMutex.lock() ) {
		act_state = state_machine.getCurrentState();
		controllerMutex.unlock();
	}
	return act_state;
}

int Controller_Multirotor::getCheckpoint() {
	int act_checkpoint;
	if ( controllerMutex.lock() ) {
		act_checkpoint = state_machine.getCheckpoint();
		controllerMutex.unlock();
	}
	return act_checkpoint;
}

int Controller_Multirotor::getTrueCheckpoint() {
	int act_truecheckpoint;
	if ( controllerMutex.lock() ) {
		act_truecheckpoint = state_machine.getTrueCheckpoint();
		controllerMutex.unlock();
	}
	return act_truecheckpoint;
}
// END: ***** Scheduler part of MyDrone *****

void Controller_Multirotor::setControlMode(Controller_MidLevel_controlMode::controlMode mode) {
	startController();

	cvg_bool error_ocurred = false;
	if ( controllerMutex.lock() ) {
		switch (mode) {
		case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
			error_ocurred = state_machine.activateTrajectoryControl();
			if (!error_ocurred) {
				midlevel_controller.setControlMode(mode);
				setAllReferences(xs, ys, yaws, zs);
			} else {
				setControlMode(MULTIROTOR_INIT_CONTROLMODE);
				controllerMutex.unlock();
				return;
			}
			break;
		case Controller_MidLevel_controlMode::POSITION_CONTROL:
			setPositionControl();
			break;
		case Controller_MidLevel_controlMode::SPEED_CONTROL:
			setSpeedControl();
			break;
		default:
			controllerMutex.unlock();
			return;
			break;
		}

		control_mode = mode;
		controllerMutex.unlock();
	}
}

Controller_MidLevel_controlMode::controlMode Controller_Multirotor::getControlMode() {

	Controller_MidLevel_controlMode::controlMode current_control_mode;
	if ( controllerMutex.lock() ) {
		current_control_mode = control_mode;
		controllerMutex.unlock();
	}
	return current_control_mode;
}

void Controller_Multirotor::setTrajectory(TrajectoryType &trajectory, TrajectoryConfiguration traj_config) {
	if ( controllerMutex.lock() ) {
		state_machine.setTrajectory(trajectory, traj_config);
		controllerMutex.unlock();
	}
}

void Controller_Multirotor::setPositionControl() {
	if ( controllerMutex.lock() ) {
		state_machine.activatePositionControl();
		midlevel_controller.setControlMode(Controller_MidLevel_controlMode::POSITION_CONTROL);
		setAllReferences(xs, ys, yaws, zs);
		controllerMutex.unlock();
	}
}

void Controller_Multirotor::setSpeedControl() {
	if ( controllerMutex.lock() ) {
		state_machine.activateSpeedControl();
		midlevel_controller.setControlMode(Controller_MidLevel_controlMode::SPEED_CONTROL);
		setAllReferences(xs, ys, yaws, zs);
		controllerMutex.unlock();
	}
}
// END: ***** Get/Accessors Controller functions *****

// ***** Logging functions *****
// [Control] events logging
Controller_Multirotor::ControlDataEvent::ControlDataEvent(cvg_ulong timeCode, double elapsed_seconds, SM_stateNames::stateNames sm_state,
		double x, double y, double vx, double vy, double yaw, double z, double vxfi, double vyfi, double dyawfi, double dzfi, double pitchfi, double rollfi)
: DroneProxy::Logs::DataWriter::Event(timeCode) {
	type = (DroneProxy::Logs::DataWriter::Event::EventType) (-1); // user event type: a type that Nacho will never use
	log_msg = 	cvgString(" [Control] Time:") 	+ elapsed_seconds;
	switch (sm_state) {
	case SM_stateNames::SPEED_CONTROL:
		log_msg += " mode:speed";
		break;
	case SM_stateNames::POSITION_CONTROL:
		log_msg += " mode:position";
		break;
	case SM_stateNames::STRAIGHT:
		log_msg += " mode:tc_straight";
		break;
	case SM_stateNames::TURN:
		log_msg += " mode:tc_turn";
		break;
	case SM_stateNames::HOVER:
		log_msg += " mode:tc_position";
		break;
	}
	log_msg +=
			cvgString(" x:")+ x +
			" y:"			+ y +
			" z:"			+ z +
			" vx:"			+ vx +
			" vy:"			+ vy +
			" vxfi:"		+ vxfi +
			" vyfi:"		+ vyfi +
			" vzfi:"		+ dzfi +
			" rollfi:"		+ rollfi +
			" pitchfi:"		+ pitchfi +
			" yaw:"			+ yaw +
			" dyawfi:"		+ dyawfi;
}

void Controller_Multirotor::saveCntRefData2Disk() {
	double xci2, yci2, vxci2, vyci2, yawci2, zci2, vxfi2, vyfi2, dzfi2, dyawfi2, pitchfi2, rollfi2;
	SM_stateNames::stateNames sm_state;

	if ( controllerMutex.lock() ) {
		midlevel_controller.getIntermediateVars( &vxco_int, &vyco_int, &yawco_int, &zco_int);
		xci2  = xci;
		yci2  = yci;
		vxci2  = vxco_int;
		vyci2  = vyco_int;
		yawci2 = yawci;
		zci2   = zci;

		vxfi2    = vxfi;
		vyfi2    = vyfi;
		dyawfi2  = dyawfi;
		dzfi2    = dzfi;
		pitchfi2 = pitchfi;
		rollfi2  = rollfi;

		sm_state = state_machine.getCurrentState();

		controllerMutex.unlock();
	}

	cvg_ulong timeCode = cvgTimer::getSystemSeconds()*1e6;
	double elapsed_seconds = pmy_drone->getTime();

	Controller_Multirotor::ControlDataEvent *control_dataevent = new Controller_Multirotor::ControlDataEvent(timeCode, elapsed_seconds, sm_state,
			xci2, yci2, vxci2, vyci2, yawci2, zci2, vxfi2, vyfi2, dzfi2, dyawfi2, pitchfi2, rollfi2);
	pmy_drone->getLogDataWritter().addEvent(control_dataevent);
	control_dataevent = NULL;
}
// END: ***** Logging functions *****
