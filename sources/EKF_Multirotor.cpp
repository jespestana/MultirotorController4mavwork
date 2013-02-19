/*
 * PelicanEKF.cpp
 *
 *  Created on: May 10, 2012
 *      Author: jespestana
 */

#include "stateObserver/EKF_Multirotor.h"
#include "MyDrone.h"

#define EKF_MAXIMUM_ADMISIBLE_SPEED 2.0 // m/s

#define PI 3.14159265
#define DEG2RAD PI/180.0

EKF_Multirotor::EKF_Multirotor(Timer &drone_clock) :
//						 cmdLog("csim_cmd_t", "csim_cmdLog.txt", "csim_cmd_P csim_cmd_R csim_cmd_dY csim_cmd_dZ"),
//                       estimatedMeasurementsLog("EKF_t", "csim_EKFLog.txt", "EKF_pitch EKF_roll EKF_dYaw EKF_Yaw EKF_dZ EKF_Z EKF_X EKF_Y EKF_Vx EKF_Vy EKF_Vxm EKF_Vym BORRAR"),
//                       cntRefLog("csim_cntRef_t", "csim_cntRefLog.txt", "csim_cntRef_x csim_cntRef_y csim_cntRef_vx csim_cntRef_vy csim_cntRef_yaw csim_cntRef_z csim_cntRef_vxfi csim_cntRef_vyfi csim_cntRef_dyawfi csim_cntRef_dzfi csim_cntRef_pitchfi csim_cntRef_rollfi"),
//                       gmLog("csim_gm_t", "csim_gmLog.txt", "csim_gm_x csim_gm_y csim_gm_yaw csim_gm_yaw_proxy csim_gm_yaw_map csim_pf_sigma_x csim_pf_sigma_y csim_pf_sigma_yaw csim_gm_z"),
//                         gmData(9),
//                         cntRefData(12),
                         EKF_started(false),
                         EKF( MULTIROTOR_MODEL_NUMSTATES, MULTIROTOR_MODEL_NUMINPUTS, MULTIROTOR_MODEL_NUMMEASURES)
{
//	debugCount = 0;

	int numStates   = MULTIROTOR_MODEL_NUMSTATES;
	int numInputs   = MULTIROTOR_MODEL_NUMINPUTS;
	int numMeasures = MULTIROTOR_MODEL_NUMMEASURES;
	pmy_drone = NULL;

	multirotorModel.setMaxIntegrationTime(0.02);
	EKF.initModel(&multirotorModel);

	//Inicializamos otras variables
	RealState.creation(numStates);
	EstimatedState.creation(numStates);
	RealActuation.creation(numInputs);
	RealActuation_km1.creation(numInputs);
	FlagObservers.creation(numMeasures);   			// Flags to be activated when a measurement is obtained
	RealObservation.creation(numMeasures);
	EstimatedObservation.creation(numMeasures);

	OdometryMeasurementsIndex.creation(6); 			// Index of odometry measurements
	OdometryMeasurementsIndex.setValueData( 1  , 1 );
	OdometryMeasurementsIndex.setValueData( 2  , 2 );
	OdometryMeasurementsIndex.setValueData( 4  , 3 );
	OdometryMeasurementsIndex.setValueData( 6  , 4 );
	OdometryMeasurementsIndex.setValueData( 11 , 5 );
	OdometryMeasurementsIndex.setValueData( 12 , 6 );

	PositionMeasurementsIndex.creation(2); 			// Index of odometry measurements
	PositionMeasurementsIndex.setValueData( 7  , 1 );
	PositionMeasurementsIndex.setValueData( 8  , 2 );

	OtherMeasurementsIndex.creation(4); 			// Index of other UNUSED measurements
	OtherMeasurementsIndex.setValueData( 3  , 1 );
	OtherMeasurementsIndex.setValueData( 5  , 2 );
	OtherMeasurementsIndex.setValueData( 9  , 3 );
	OtherMeasurementsIndex.setValueData( 10 , 4 );

	/////// Observation variances
	VarObservation_EKF.creation(numMeasures);
	// Odometry measurements
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_PITCH_DEG 	*PI/180.0)      , 2 ) ,1); // Pitch
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_ROLL_DEG 	*PI/180.0)      , 2 ) ,2); // Roll
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_YAW_ODOMETRY_DEG*PI/180.0)  , 2 ) ,4); // Yaw
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_Z_M					 )      , 2 ) ,6); // Z
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_VXM_MPS				 )      , 2 ) ,11);// Vxm
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_VYM_MPS 			 )      , 2 ) ,12);// Vym
	// Position measurements
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_X_M					 )      , 2 ) ,7); // X position (PX)
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_Y_M					 )      , 2 ) ,8); // Y position (PY)
	VarObservation_EKF.setValueData( pow( ( STD_OBS_EKF_YAW_VICON_DEG*PI/180.0)     , 2 ) ,13); // Yaw
	// Other measurements
	VarObservation_EKF.setValueData( STD_OBS_EKF_UNUSED ,3); // dYaw/dt
	VarObservation_EKF.setValueData( STD_OBS_EKF_UNUSED ,5); // dZ/dt
	VarObservation_EKF.setValueData( STD_OBS_EKF_UNUSED ,9); // Vx
	VarObservation_EKF.setValueData( STD_OBS_EKF_UNUSED ,10);// Vy

	/////// Command variances
	VarActuation_EKF.creation(numInputs);
	VarActuation_EKF.setValueData( pow( ( STD_ACT_EKF_PITCH_DEG		/24)   , 2 ) ,1);  // Pitch
	VarActuation_EKF.setValueData( pow( ( STD_ACT_EKF_ROLL_DEG 		/24)   , 2 ) ,2);  // Roll
	VarActuation_EKF.setValueData( pow( ( STD_ACT_EKF_DYAW_DEGPS	/100)  , 2 ) ,3);  // dYaw/dt
	VarActuation_EKF.setValueData( pow( ( STD_ACT_EKF_DZ_MPS		/1)    , 2 ) ,4);  // dZ/dt

	/////// Initial state estimation variances
	varStateModel_EKF.creation(numStates);
	varStateModel_EKF.setValueData( pow( STD_STM_EKF_YAW_DEG*PI/180.0 ,2) , 5);
	varStateModel_EKF.setValueData( pow( STD_STM_EKF_Z_M	 ,2) , 8);
	varStateModel_EKF.setValueData( pow( STD_STM_EKF_X_M	 ,2) , 9);
	varStateModel_EKF.setValueData( pow( STD_STM_EKF_Y_M	 ,2) ,10);
	varStateModel_EKF.setValueData( pow( STD_STM_EKF_VX_MPS	 ,2) ,11);
	varStateModel_EKF.setValueData( pow( STD_STM_EKF_VY_MPS	 ,2) ,12);

	MatPInit.creation(numStates,numStates);
	MatPInit.setValueData( pow( (( STD_IST_EKF_PITCH_DEG  *PI/180)/58.227) , 2 ) ,1 ,1 ); //  X1 : proportional to pitch
	MatPInit.setValueData( pow( (( STD_IST_EKF_ROLL_DEG   *PI/180)/38.575) , 2 ) ,2 ,2 ); //  X2 : proportional to roll
	MatPInit.setValueData( pow( (( STD_IST_EKF_DYAW1_DEGPS*PI/180)/67.213) , 2 ) ,3 ,3 ); //  X3 : internal variable related to d(yaw)/dt
	MatPInit.setValueData( pow( (( STD_IST_EKF_DYAW2_DEGPS*PI/180)/67.213) , 2 ) ,4 ,4 ); //  X4 : proportional to d(yaw)/dt
	MatPInit.setValueData( pow( (( STD_IST_EKF_YAW_DEG    *PI/180)       ) , 2 ) ,5 ,5 ); //  X5 : yaw
	MatPInit.setValueData( pow( (  STD_IST_EKF_DZ1_MPS   /14.189)          , 2 ) ,6 ,6 ); //  X6 : internal variable related to d(Z)/dt
	MatPInit.setValueData( pow( (  STD_IST_EKF_DZ2_MPS   /14.189)          , 2 ) ,7 ,7 ); //  X7 : proportional to d(Z)/dt
	MatPInit.setValueData( pow( (  STD_IST_EKF_Z_M       )                 , 2 ) ,8 ,8 ); //  X8 : Z
	MatPInit.setValueData( pow( (  STD_IST_EKF_X_M       )                 , 2 ) ,9 ,9 );	//  X9 : X
	MatPInit.setValueData( pow( (  STD_IST_EKF_Y_M       )                 , 2 ) ,10,10);	//  X10: Y
	MatPInit.setValueData( pow( (  STD_IST_EKF_VX_MPS    )                 , 2 ) ,11,11);	//  X11: VX
	MatPInit.setValueData( pow( (  STD_IST_EKF_VY_MPS    )                 , 2 ) ,12,12);	//  X12: VY

	EKF.init(&EstimatedState,&VarActuation_EKF,&VarObservation_EKF,&MatPInit);
	EKF.setVarState(&varStateModel_EKF);
	EKF.setInitTime( 0.0 );
}

EKF_Multirotor::~EKF_Multirotor() {
}

//void EKF_Multirotor::resetEKF() {
//
//	if (!started_once) {
//		if ( EKF_Mutex.lock() ) {
//			timer.restart(false);
//			EKF.setInitTime( 0.0 );
//			EKF_Mutex.unlock();
//		}
//		started_once = true;
//	}
//
//	started = false;
//
//}

void EKF_Multirotor::setSystemOdometryMeasures(FeedbackData *feedbackData) {


	//	struct feedbackData__ {
	//			unsigned char	grantedAccessMode;
	//			unsigned int	timeCodeH;
	//			unsigned int	timeCodeL;
	//			char			commWithDrone;
	//			char		 	droneMode;
	//			int				nativeDroneState;
	//			float			batteryLevel;
	//			float			roll;
	//			float			pitch;
	//			float			yaw;
	//			float			altitude;
	//			float			speedX;
	//			float			speedY;
	//			float			speedYaw;
	//		} __PACKED feedbackData;

	//  Medidas:
	//  01 pitch: C(1,1)*X1
	//  02 roll : C(2,2)*X2
	//  03 dYaw : C(3,4)*X4
	//  04 Yaw  : X5
	//  05 dZ   : C(5,7)*X7
	//  06 Z    : X8
	//  07 X    : X9					"VISION"
	//  08 Y    : X10					"VISION"
	//  09 Vx   : X11
	//  10 Vy   : X12
	//  11 [Vxm;: [R_Y R_Y]' * [X11;
	//  12 Vym] : [R_Y R_Y]     X12]
	//  13 Yaw  : X5					"VISION"

	double actualTime = pmy_drone->getTime();

	if (EKF_Mutex.lock()) {

		// make the estimation advance until the actualTime
		for ( int i = 1; i <= MULTIROTOR_MODEL_NUMMEASURES; i ++) {
			EKF.deactivateSystemMeasure(i);
			FlagObservers.setValueData( 0.0, i);
		}
		stateEstimation(PARROTEKF_MAHALANOBIS_DISTANCE, false);

		cvg_double pitch_rad = feedbackData->pitch*DEG2RAD;
		cvg_double roll_rad  = feedbackData->roll*DEG2RAD;

		cvg_double yaw_rad   = feedbackData->yaw*DEG2RAD;
		EKF.getEstimatedState(&EstimatedState);
		multirotorModel.observationModel(&EstimatedObservation, &EstimatedState);
		double yaw_EKF_rad = EstimatedObservation.getValueData( 4 );
		yaw_EKF_rad = jesus_library::mapAnglesToBeNear_PIrads( yaw_EKF_rad, yaw_rad);
		EstimatedState.setValueData(yaw_EKF_rad, 5);
		EKF.setInitialState(&EstimatedState);

		float vx_fb = feedbackData->speedX, vy_fb = feedbackData->speedY;
		float v = sqrt(vx_fb*vx_fb + vy_fb*vy_fb);
		// feedbackData->commWithDrone == true -> Communicaciones ok
		// feedbackData->commWithDrone == false -> perdida de comunicacion, medidas potencialmente erroneas
		if ( (feedbackData->commWithDrone) & (v <= EKF_MAXIMUM_ADMISIBLE_SPEED)) {
			RealObservation.setValueData( feedbackData->speedX, 11);
			FlagObservers.setValueData( 1.0, 11 );
			RealObservation.setValueData( feedbackData->speedY, 12);
			FlagObservers.setValueData( 1.0, 12 );
			RealObservation.setValueData( yaw_rad    , 4);  // Hay que arreglar yaw_rad
			FlagObservers.setValueData( 1.0, 4 );
			RealObservation.setValueData( pitch_rad  , 1);
			FlagObservers.setValueData( 1.0, 1 );
			RealObservation.setValueData( roll_rad   , 2);
			FlagObservers.setValueData( 1.0, 2 );
			RealObservation.setValueData( feedbackData->altitude,6);
			FlagObservers.setValueData( 1.0, 6 );
			EKF.setSystemMeasures(&RealObservation);
		} else {
			FlagObservers.setValueData( 0.0, 11 );
			FlagObservers.setValueData( 0.0, 12 );
			FlagObservers.setValueData( 0.0, 4 );
			FlagObservers.setValueData( 0.0, 1 );
			FlagObservers.setValueData( 0.0, 2 );
			FlagObservers.setValueData( 0.0, 6 );
		}

		stateEstimation(PARROTEKF_MAHALANOBIS_DISTANCE);
		EKF_Mutex.unlock();
	}



}

int EKF_Multirotor::stateEstimation(float mahalanobisDistance, bool log_data) {

	int output = 0;
	cvg_ulong timeCode = cvgTimer::getSystemSeconds()*1e6;
	double actualTime = pmy_drone->getTime(); // timer.getElapsedSeconds();

//	timer.restart(started);
	if (!EKF_started) {
		for (int i = 1; i <= EstimatedState.length(); i++ ) {
			EstimatedState.setValueData( 0.0, i);
		}

		EKF.init(&EstimatedState,&VarActuation_EKF,&VarObservation_EKF,&MatPInit);
		EKF.setVarState(&varStateModel_EKF);

		EKF_started = true;
		return output; // MIRAR ESTO!!!
	}


	double last_actualTime =  EKF.getActualTime();
	if (last_actualTime < actualTime) {
		EKF.setActualTime(actualTime);
	}

	//Activa o desactiva observacion
	for( int i=1; i<=MULTIROTOR_MODEL_NUMMEASURES; i++) {
		if ( FlagObservers.getValueData(i) == 1.0 ) {
			EKF.activateSystemMeasure(i);
			FlagObservers.setValueData(0.0,i);
		} else {
			EKF.deactivateSystemMeasure(i);
		}
	}

	output = EKF.stateEstimation(mahalanobisDistance);

	if (log_data) {
		EKF.getEstimatedState(&EstimatedState);
		multirotorModel.observationModel(&EstimatedObservation, &EstimatedState);
		EKF_Multirotor::EKFDataEvent *ekf_dataevent = new EKF_Multirotor::EKFDataEvent(timeCode, actualTime, EstimatedObservation);
		pmy_drone->getLogDataWritter().addEvent(ekf_dataevent);
		ekf_dataevent = NULL;
	}

	return output;
}

void EKF_Multirotor::getEstimatedObservation(Vector *estObserv) {

	if ( EKF_Mutex.lock() ) {
		stateEstimation( PARROTEKF_MAHALANOBIS_DISTANCE );

		EKF.getEstimatedState(&EstimatedState);
		multirotorModel.observationModel( estObserv, &EstimatedState);

		EKF_Mutex.unlock();
	}
}

void EKF_Multirotor::getEstimatedQuadrotorPosition( double &x, double &y, double &z, double &yaw) {

	if ( EKF_Mutex.lock() ) {
		getEstimatedObservation(&EstimatedObservation);

		x = EstimatedObservation.getValueData(7);
		y = EstimatedObservation.getValueData(8);
		z = EstimatedObservation.getValueData(6);
		yaw = EstimatedObservation.getValueData(4);

		EKF_Mutex.unlock();
	}

}

// ***** Functions that act on the input to the system *****
void EKF_Multirotor::systemInputsSet() {

	//	// Assign inputs
	//	for (int i = 1; i <= inputsIn->length(); i++) {
	//		RealActuation.setValueData( inputsIn->getValueData(i), i);
	//	}

	double actualTime = pmy_drone->getTime(); // timer.getElapsedSeconds();

	if ( EKF_Mutex.lock() ) {
		stateEstimation(PARROTEKF_MAHALANOBIS_DISTANCE);
		EKF.getSystemInputs(&RealActuation_km1);
//		logCommandData(actualTime, &RealActuation_km1);
		EKF.setSystemInputs(&RealActuation);
//		logCommandData(actualTime, &RealActuation);
		EKF_Mutex.unlock();
	}
}

void EKF_Multirotor::setInputPitch(cvg_float pitch) {
	if ( EKF_Mutex.lock() ) {
		RealActuation.setValueData( pitch, 1);  // 1
		EKF_Mutex.unlock();
	}
}
void EKF_Multirotor::setInputRoll(cvg_float roll)   {
	if ( EKF_Mutex.lock() ) {
		RealActuation.setValueData( roll,  2);  // 2
		EKF_Mutex.unlock();
	}
}
void EKF_Multirotor::setInputdYaw(cvg_float dyaw)   {
	if ( EKF_Mutex.lock() ) {
		RealActuation.setValueData( dyaw,  3);  // 3
		EKF_Mutex.unlock();
	}
}
void EKF_Multirotor::setInputGaz(cvg_float gaz)     {
	if ( EKF_Mutex.lock() ) {
		RealActuation.setValueData( gaz,   4); 	// 4
		EKF_Mutex.unlock();
	}
}
// END: ***** Functions that act on the input to the system *****

// ***** Logging functions *****

// EKFDataEvent
EKF_Multirotor::EKFDataEvent::EKFDataEvent(cvg_ulong timeCode, double elapsed_seconds, Vector &EstimatedObservation)
				: DroneProxy::Logs::DataWriter::Event(timeCode) {
	//  Medidas:
	//  01 pitch: C(1,1)*X1
	//  02 roll : C(2,2)*X2
	//  03 dYaw : C(3,4)*X4
	//  04 Yaw  : X5
	//  05 dZ   : C(5,7)*X7
	//  06 Z    : X8
	//  07 X    : X9					"VISION"
	//  08 Y    : X10					"VISION"
	//  09 Vx   : X11
	//  10 Vy   : X12
	//  11 [Vxm;: [R_Y R_Y]' * [X11;
	//  12 Vym] : [R_Y R_Y]     X12]
	//  13 Yaw  : X5					"VISION"
	type = (DroneProxy::Logs::DataWriter::Event::EventType) (-1); // user eventtype: a type that Nacho will never use
	//	("EKF_t", "csim_EKFLog.txt", "EKF_pitch EKF_roll EKF_dYaw EKF_Yaw EKF_dZ EKF_Z EKF_X EKF_Y EKF_Vx EKF_Vy EKF_Vxm EKF_Vym BORRAR");
	log_msg = 	cvgString(" [EKF] Time:") 	+ elapsed_seconds +
			" pitch:"			+ EstimatedObservation.getValueData(1) +
			" roll:"				+ EstimatedObservation.getValueData(2) +
			" dYaw:"				+ EstimatedObservation.getValueData(3) +
			" Yaw:"				+ EstimatedObservation.getValueData(4) +
			" dZ:"				+ EstimatedObservation.getValueData(5) +
			" Z:"				+ EstimatedObservation.getValueData(6) +
			" X:"				+ EstimatedObservation.getValueData(7) +
			" Y:"				+ EstimatedObservation.getValueData(8) +
			" Vx:"				+ EstimatedObservation.getValueData(9) +
			" Vy:"				+ EstimatedObservation.getValueData(10) +
			" Vxm:"				+ EstimatedObservation.getValueData(11) +
			" Vym:"				+ EstimatedObservation.getValueData(12);
}
// END: ***** Logging functions *****
