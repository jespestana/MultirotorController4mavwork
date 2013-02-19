/*
 * MyDrone.cpp
 *
 *  Created on: 13/11/2012
 *      Author: Jesus Pestana
 */

#include "MyDrone.h"

#define DEG2RAD (M_PI  / 180.0)
#define RAD2DEG (180.0 / M_PI )

using namespace DroneProxy;
using namespace cv;

MyDrone::MyDrone() : drone_clock(), state_observer(drone_clock) {

	// BGN: Basic Proxy constructor
	cvStartWindowThread();
	cvNamedWindow("camera_0");
	frame = NULL;

	positioningSource = POSITIONING_ODOMETRY;
	// END: Basic Proxy constructor

	// ***** Common Resources *****
	drone_clock.restart(false);
	// ***** END: Common Resources *****

	// ***** Controller *****
	controller.setMyDrone(this);
//	controller.setControlMode(MULTIROTOR_INIT_CONTROLMODE); // not a good idea
	// ***** END: Controller *****

	// ***** State estimation *****
	// state_observer(drone_clock); // See initialization list
	state_observer.setMyDrone(this);
	state_obs_started = false;
	// ***** END: State estimation *****
}

MyDrone::~MyDrone() {
	close();

	cvReleaseImageHeader(&frame);
	frame = NULL;
	cvDestroyWindow("camera_0");
}

void MyDrone::setConfigParams() {
	LedsAnimation anim;
	anim.typeId = LEDS_RED_SNAKE;
	anim.frequencyHz = 2.0f;
	anim.durationSec = 0;
	writeParam(CONFIGPARAM_LEDS_ANIMATION, (cvg_char *)&anim, sizeof(LedsAnimation));
	// Usual parrot configuration values
	writeParam(CONFIGPARAM_MAX_EULER_ANGLES, 10.0f); 	// deg, max value around 24 deg
	writeParam(CONFIGPARAM_MAX_VERTICAL_SPEED, 1.0f);	// m/s
	writeParam(CONFIGPARAM_MAX_YAW_SPEED, 100.0f);		// deg/s
	writeParam(CONFIGPARAM_VIDEO_ENCODING_TYPE, JPEG);
	writeParam(CONFIGPARAM_VIDEO_ENCODING_QUALITY, 1.0f);
}

void MyDrone::processVideoFrame(cvg_int cameraId, cvg_ulong timeCode, VideoFormat format, cvg_uint width, cvg_uint height, cvg_char *frameData) {

	// OpticalFlow camera: cameraId 0, width 376, height 240
	// Hokuyo			 : cameraId 1, width 726, height   1
	// std::cout << "cameraId = " << cameraId << "; ";
	// std::cout <<  "width = " << width << "; ";
	// std::cout << "height = " << height << "\n";

	switch (cameraId) {
	case 0: // OpticalFlow camera
		if (frame == NULL) {
			frame = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
		}
		cvSetData(frame, frameData, width * 3);
		cvShowImage("camera_0", frame);

		// Save data to log
		LoggerDrone::processVideoFrame(cameraId, timeCode, format, width, height, frameData);
		break;
	case 1: // Hokuyo laser range finder
		break;
	}
}

void MyDrone::processFeedback(FeedbackData *feedbackData0) {
//  FeedbackData struct declaration information
//	struct feedbackData__ {
//		unsigned char	grantedAccessMode;
//		unsigned int	timeCodeH;
//		unsigned int	timeCodeL;
//		char			commWithDrone;
//		char		 	droneMode;
//		int				nativeDroneState;
//		float			batteryLevel;
//		float			roll;
//		float			pitch;
//		float			yaw;
//		float			altitude;
//		float			speedX;
//		float			speedY;
//		float			speedYaw;
//	} __PACKED feedbackData;


	LoggerDrone::processFeedback(feedbackData0);
	if (!feedbackData0->commWithDrone) // If the communication with the drone is lost, only the [info] data is logged
		return;

	FeedbackData feedbackDataCopy;
	FeedbackData *feedbackData = &feedbackDataCopy;
	memcpy(feedbackData, feedbackData0, sizeof(FeedbackData));

	feedbackData->altitude = (-1)*feedbackData->altitude;

//	log(cvgString("state_obs_started: ")  + state_obs_started +
//			"\nfeedbackData->droneMode == HOVER: "  + (feedbackData->droneMode == HOVER) +
//			"\nfeedbackData->droneMode == FLYING: " + (feedbackData->droneMode == FLYING) +
//			"\ngetControlMode() == MOVE: "          + (getControlMode() == MOVE));
	if (!state_obs_started) { // && feedbackData->droneMode == HOVER && ( feedbackData->droneMode != FLYING || getControlMode() != MOVE ) ) {
		return;
	}
//	log("************* state_observer.setSystemOdometryMeasures(feedbackData); ********************");
	state_observer.setSystemOdometryMeasures(feedbackData);
}

cvg_bool MyDrone::setControlData( cvg_float phi, cvg_float theta, cvg_float gaz, cvg_float yawSpeed) {
	cvg_int changes = Comm::ControlChannel::THETA | Comm::ControlChannel::PHI | Comm::ControlChannel::YAW | Comm::ControlChannel::GAZ;
	return setControlData(changes, phi, theta, gaz, yawSpeed);
}


cvg_bool MyDrone::setControlData(cvg_int changes, cvg_float phi, cvg_float theta, cvg_float gaz, cvg_float yawSpeed) {

	FlyingMode actualFlyingMode = getControlMode();
	if ( actualFlyingMode == HOVER ) {
		phi 	 = 0.0;
		theta 	 = 0.0;
		gaz 	 = 0.0;
		yawSpeed = 0.0;
	}

	cvg_bool result = LoggerDrone::setControlData(changes, phi, theta, gaz, yawSpeed);
	return result;

}

cvg_bool MyDrone::setControlMode(FlyingMode mode) {

	// Posible values for FlyingMode: HOVER, MOVE, EMERGENCYSTOP, LAND, TAKEOFF
	FlyingMode mode_before = getControlMode();
	switch (mode) {
	case LAND:
	case EMERGENCYSTOP:
	case HOVER:
	case TAKEOFF:
		controller.stopController();
	break;
	case MOVE:
		if ( mode_before != MOVE) {
			controller.stopController();
		}
		break;
	}

	return LoggerDrone::setControlMode(mode);
}


