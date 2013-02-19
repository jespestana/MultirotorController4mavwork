/*
 * PelicanEKF.h
 *
 *  Created on: May 10, 2012
 *      Author: jespestana
 */

#ifndef MULTIROTOR_EKF_H_
#define MULTIROTOR_EKF_H_

// Which multirotor is going to be controlled?
#include "config_Mydrone.h"

#include "stateObserver/EKF_lib/extendedKalmanFilter.h"
#include "stateObserver/EKF_lib/matrixLib.h"

#ifdef _MULTIROTOR_IS_PARROT_
#include "stateObserver/models/EKF_config_Parrot.h"
#include "stateObserver/models/EKF_model_Parrot.h"
typedef ParrotModel1 Multirotor_Model;
#endif
#ifdef _MULTIROTOR_IS_PELICAN_
#include "stateObserver/models/EKF_config_Pelican.h"
#include "stateObserver/models/EKF_model_Pelican.h"
typedef PelicanModel1 Multirotor_Model;
#endif
//#include "datalogger.h"

// to-do list:
// TODO: Cosas que me gustaría añadir al EKF,
//			- TODO: Hacer pruebas con diferentes constantes para las varianzas de medida, de ruido a la entrada, etc
//			- TODO: habilitar/permitir cambios de configuracion del EKF online

#include "Other/jesus_library.h"
#include "Other/Timer.h"
#include <math.h>

//#include "MyDrone.h"
class MyDrone;
#include <droneproxy.h>

class EKF_Multirotor {
private:
	Multirotor_Model				multirotorModel;

	// ***** Internal objects and variables *****
	ContinuousExtendedKalmanFilter 	EKF;
	volatile bool EKF_started;
	DroneProxy::Threading::Mutex EKF_Mutex;
	Vector RealState;
	Vector EstimatedState;
	Vector FlagObservers;   			// Flags to be activated when a measurement is obtained
	Vector RealObservation;
	// This two vector may be used for logging
	Vector RealActuation; 	 			// Without noise
	Vector RealActuation_km1;


	Vector OdometryMeasurementsIndex; 	// Index of odometry measurements
	Vector PositionMeasurementsIndex; 	// Index of odometry measurements
	Vector OtherMeasurementsIndex; 		// Index of other UNUSED measurements

	Vector VarObservation_EKF;
	Vector VarActuation_EKF;
	Vector varStateModel_EKF;
	Matrix MatPInit;
	// END: ***** Internal objects and variables *****

	Vector EstimatedObservation;

	// Function to move forward in time the state estimation
	int stateEstimation(float mahalanobisDistance, bool log_data = true);
	MyDrone *pmy_drone; // pointer to my_drone: timer, and logging through events

public:
	EKF_Multirotor(Timer &drone_clock);
	void setMyDrone(MyDrone *p2my_drone) {pmy_drone = p2my_drone;} // use in MyDrone constructor
	~EKF_Multirotor();

//	void resetEKF(); // Creo que no hace falta, 15 Nov 2012

	inline void setModelIntputGains(double gain_pitch, double gain_roll, double gain_dyaw, double gain_dz) {
		multirotorModel.setIntputGains(gain_pitch, gain_roll, gain_dyaw, gain_dz);
	}

	void systemInputsSet();
	void setInputPitch(cvg_float pitch);
	void setInputRoll(cvg_float roll);
	void setInputdYaw(cvg_float dyaw);
	void setInputGaz(cvg_float gaz);

	void setSystemOdometryMeasures(FeedbackData *feedbackData);
	void getEstimatedObservation(Vector *estObserv);
	void getEstimatedQuadrotorPosition( double &x, double &y, double &z, double &yaw);

private:
	// ***** [EKF] datalogging *****
	class EKFDataEvent : public DroneProxy::Logs::DataWriter::Event {
	private:
		cvgString log_msg;
	public:
		EKFDataEvent(cvg_ulong timeCode, double elapsed_seconds, Vector &EstimatedObservation);
		virtual inline ~EKFDataEvent() { }
		virtual inline cvgString toString() { return log_msg; }
	};
	// END: ***** [EKF] datalogging *****
};


#endif /* MULTIROTOR_EKF_H_ */
