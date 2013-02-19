///*
// * task_scheduler.h
// *
// *  Created on: Jun 29, 2012
// *      Author: jespestana
// */
//
//#ifndef TASK_SCHEDULER_H_
//#define TASK_SCHEDULER_H_
//
//#include "atlante.h"
//#include "PelicanEKF.h"
//#include "config_PelicanEKF.h"
//#include <droneproxy.h>
//#include <math.h>
//
//class MyDrone;
//
//class Task_scheduler : public virtual cvgThread {
//private:
//	volatile cvg_bool reset_flag;
//	volatile cvg_bool planifier_started;
//
//	DroneProxy::Threading::Mutex planifierMutex;
//	DroneProxy::Timing::Timer timer;
//
//	PelicanEKF *p2pelicanEKF;
//	Vector estObserv;
//
//	MyDrone *myDrone;
//
//	double x_ekf, y_ekf, z_ekf, yaw_ekf;
//	double desired_yaw;
//
//	void reset();
//
//protected:
//	void run();
//
//public:
//	Task_scheduler(PelicanEKF *pelicanEKF);
//	~Task_scheduler();
//
//	void setMyDrone(MyDrone *drone);
//
//private:
//	// Pylon Challenge tasks
//	void takeoffTask();
//	void searchPylons1();
//	void makeHalfEightPylons1();
//	void yawTurnToZero();
//	void searchPylons2();
//	void yawTurnToMinusPI();
//	void makeHalfEightPylons2();
//	void finalLandingTask();
//
//	// Landing if reset_flag == true;
//	inline void whileLoopMandatoryCode();
////	void landingDueToResetTask();
//
//	inline void waitSeconds(float seconds2wait)    {	usleep( seconds2wait*1.0E6 );  }
//
//	cvgString debugString;
//
//public:
//	inline void startPlanifier() { planifier_started = true; }
//
//	inline cvgString *getDebugString() {
//		cvgString *actDebugString;
//		if (planifierMutex.lock()) {
//			actDebugString = &debugString;
//			planifierMutex.unlock();
//		}
//		return actDebugString;
//	}
//
//};
//
//
//
//#endif /* TASK_SCHEDULER_H_ */
