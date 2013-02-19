/*
 * MyDrone.h
 *
 *  Created on: 13/11/2012
 *      Author: Jesus Pestana
 */

#ifndef MYDRONE_H_
#define MYDRONE_H_

// Proxy and proxy libraries
#include <droneproxy.h>
#include <atlante.h>

// Camera image window
#include <opencv/cv.h>		// changed to make it work with ubuntu 12.04 opencv software center installaion
//#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// State Observer
#include "stateObserver/EKF_Multirotor.h"
typedef EKF_Multirotor StateObserver;

// Controller
#include "controller/Controller_Multirotor.h"
typedef Controller_Multirotor Controller;

// Other resources
#include "Other/Timer.h"

// to-do list:
// TODO: ensure that the following components use the same configuration constants:
//		- Model used inside the EKF
//		- configParams function which configures the multirotor vehicle
//		- controller
//		- Take into account that until now each vehicle is preconfigured and I never change the basic config

class MyDrone : public virtual DroneProxy::LoggerDrone {

	// ***** Basic proxy declarations *****
private:
	IplImage *frame; // used in MyDrone::processVideoFrame(...)
public:
	typedef enum { POSITIONING_VICON, POSITIONING_ODOMETRY } PositioningSource ;
private:
	PositioningSource positioningSource;
protected:
	virtual void processVideoFrame(cvg_int cameraId, cvg_ulong timeCode, VideoFormat format, cvg_uint width, cvg_uint height, cvg_char *frameData);
	virtual void processFeedback(FeedbackData *feebackData);
	virtual void setConfigParams();
public:
	MyDrone();
	~MyDrone();

	// Overriden from LoggerDrone
	cvg_bool setControlMode(FlyingMode m);
	// Overriden setControlData
	cvg_bool setControlData(cvg_int changes, cvg_float phi, cvg_float theta, cvg_float gaz, cvg_float yawSpeed);
	cvg_bool setControlData(cvg_float phi, cvg_float theta, cvg_float gaz, cvg_float yawSpeed);
	// ***** END: Basic proxy declarations *****

	// ***** Common Resources *****
private: Timer drone_clock;
public:  inline double getTime() { return drone_clock.getElapsedSeconds(); }
	// ***** END: Common Resources *****

	// ***** Controller declarations *****
    Controller controller;
	// ***** END: Controller declarations *****

	// ***** State estimation declarations *****
private: volatile cvg_bool state_obs_started;
public:  StateObserver state_observer;
		 void startStateObsever() { state_obs_started = true; }
	// ***** END: State estimation declarations *****
};

#endif /* MYDRONE_H_ */
