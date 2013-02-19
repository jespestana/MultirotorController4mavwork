#ifndef _PELICAN_MODEL_H
#define _PELICAN_MODEL_H


#include "stateObserver/EKF_lib/model.h"
//#include "jesus_library.h"

// Integration timestep
// extern float timeIntegration;

// Saturation of model input in pitch, roll, dyaw and dz
//#define LIM_PITCH_REF      0.5
//#define LIM_ROLL_REF       0.5
//#define LIM_DYAW_REF       0.5
//#define LIM_DZ_REF         0.5

//First model
class PelicanModel1 : public ContinuousModel {
	// Valores por defecto durante 2012:
	//		pelicanEKF.setModelIntputGains( 9.0*M_PI/180.0, 9.0*M_PI/180.0, 100.0*M_PI/180.0, +0.5);
	//		Input_gains = diag([gain_pitch, gain_roll, gain_dyaw, gain_dz]);
private:
	double gain_pitch, gain_roll, gain_dyaw, gain_dz;

public:
	// Default constructor
	PelicanModel1();
	//User process model
	virtual void processModel(Vector* Statek1, Vector* Statek, Vector* Inputs);
	//User Observation model
	virtual void observationModel(Vector* Output, Vector* State);
	//User Jacobian process model
	virtual void jacobiansProcessModel(Matrix* MatJacFx, Matrix* MatJacFu, Vector* State, Vector* Inputs);
	//User Jacobian process model
	virtual void jacobiansObservationModel(Matrix* MatJacHx, Vector* State);
	virtual ~PelicanModel1() {}

	inline void setIntputGains(double gain_pitch, double gain_roll, double gain_dyaw, double gain_dz) {
		this->gain_pitch = gain_pitch;
		this->gain_roll  = gain_roll;
		this->gain_dyaw  = gain_dyaw;
		this->gain_dz    = gain_dz;
	}
};



#endif
