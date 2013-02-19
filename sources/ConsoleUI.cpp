/*
 * ConsoleUI.cpp
 *
 *  Created on: 15/08/2011
 *      Author: Ignacio Mellado
 *  Modified  : 31/10/2012
 *      Author: Jesus Pestana Puerta
 */

#include <ncurses.h>
#include <ConsoleUI.h>
#include <atlante.h>
#include <math.h>
#include <string.h>
#include <droneproxy.h>
#include <iostream>

using namespace DroneProxy;
using namespace DroneProxy::Comm;

// move non-controlled UI commands
#define TILT_COMMAND_FREEMODE		 			0.50f
#define YAW_COMMAND_FREEMODE					1.00f
#define GAZ_COMMAND_FREEMODE					0.30f

// speed-controlled UI commands
#define SPEED_COMMAND_SPEEDMODE					0.50f 	// m/s
#define DELTA_ALTITUDE_COMMAND_SPEEDMODE		0.30f	// m
#define DELTA_YAW_COMMAND_SPEEDMODE				( 10 * M_PI / 180.0 ) // (def)*(deg->rad)

// position-controlled UI commands
#define DELTA_POSITION_COMMAND_POSITIONMODE		1.0f	// m
#define DELTA_ALTITUDE_COMMAND_POSITIONMODE		DELTA_ALTITUDE_COMMAND_SPEEDMODE
#define DELTA_YAW_COMMAND_POSITIONMODE			DELTA_YAW_COMMAND_SPEEDMODE

// trajectory-controlled UI commands
#define DELTA_YAW_COMMAND_TRAJECTORYMODE		DELTA_YAW_COMMAND_POSITIONMODE

/* ***** Movement commands ***** */
//			case KEY_UP: 	// move forward
//			case KEY_DOWN:	// move backward
//			case KEY_LEFT:	// move left
//			case KEY_RIGHT:	// move right
//			case 's': // 'stop' set pitch,roll,dyaw,dz parrot direct commands to 0.0
//			case 'a': // go down
//			case 'q': // go up
//			case 'z': // turn leftwards
//			case 'x': // turn rightwards

/* ***** Multirotor parrot-like control modes ***** */
//			case 'h': // hover
//			case 'm': // move
//			case ' ': // emergency step
//			case 'l': // land
//			case 't': // take off

/* ***** Multirotor modules: stateObserver, Controller ***** */
//			case 'k': // start state observer (EKF)
//			case 'v': // speed control
//			case 'b': // position control
//			case 'n': // trajectory control

ConsoleUI::ConsoleUI(MyDrone *drone) {
	this->drone = drone;
	isInit = false;
	drone->addLogConsumer(this);
}

ConsoleUI::~ConsoleUI() {
	destroy();
	drone->removeLogConsumer(this);
}

void ConsoleUI::init() {
	if (consoleMutex.lock()) {
		if (!isInit) {
			isInit = true;

			initscr();
			raw();
			keypad(stdscr, true);
			noecho();
			nodelay(stdscr, true);

			// Log window creation
			int row, col;
			getmaxyx(stdscr, row, col);
			logWin = newwin(row / 2, col, row / 2, 0);
			box(logWin, 0, 0);
			scrollok(logWin, true);
			wmove(logWin, 1, 0);
			refresh();
			wrefresh(logWin);

			memset(&lastDroneInfo, 0, sizeof(lastDroneInfo));

		}
		consoleMutex.unlock();
		drone->getChannelManager().getFeedbackChannel().addEventListener(this);
	} else throw cvgException("[ConsoleUI::init] Cannot lock console mutex");

	logConsume(NULL, "User interface ready");
}

void ConsoleUI::destroy() {
	if (consoleMutex.lock()) {
		if (isInit) {

			drone->getChannelManager().getFeedbackChannel().removeEventListener(this);

			// Destroy log window
			wborder(logWin, ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ');
			wrefresh(logWin);
			delwin(logWin);

			endwin();

			isInit = false;

			printf("[ConsoleUI] User interface closed\n");
		}
		consoleMutex.unlock();
	}
}



void ConsoleUI::doLoop() {

	bool loop = true;
	while(loop) {
		refreshInfo();
		Controller_MidLevel_controlMode::controlMode control_mode = drone->controller.getControlMode();
		cvg_double xc = 0.0, yc = 0.0, yawc = 0.0, zc = 0.0;
		switch(getch()) {
		case 27:
			loop = false;
			break;
			/* ***** Movement commands ***** */
		case KEY_UP: 	// move forward
			if (!drone->controller.isStarted())
				drone->setControlData(ControlChannel::THETA, 0.0f, -TILT_COMMAND_FREEMODE, 0.0f, 0.0f);
			else {
				switch (control_mode) {
				case Controller_MidLevel_controlMode::SPEED_CONTROL:
					drone->controller.setVXFIReference( +SPEED_COMMAND_SPEEDMODE);
					break;
				case Controller_MidLevel_controlMode::POSITION_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setXReference( xc + DELTA_POSITION_COMMAND_POSITIONMODE);
					break;
				case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
					break;
				}
			}
			break;
		case KEY_DOWN:	// move backward
			if (!drone->controller.isStarted())
				drone->setControlData(ControlChannel::THETA, 0.0f,  TILT_COMMAND_FREEMODE, 0.0f, 0.0f);
			else {
				switch (control_mode) {
				case Controller_MidLevel_controlMode::SPEED_CONTROL:
					drone->controller.setVXFIReference( -SPEED_COMMAND_SPEEDMODE);
					break;
				case Controller_MidLevel_controlMode::POSITION_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setXReference( xc - DELTA_POSITION_COMMAND_POSITIONMODE);
					break;
				case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
					break;
				}
			}
			break;
		case KEY_LEFT:	// move left
			if (!drone->controller.isStarted())
				drone->setControlData(ControlChannel::PHI,  -TILT_COMMAND_FREEMODE, 0.0f, 0.0f, 0.0f);
			else {
				switch (control_mode) {
				case Controller_MidLevel_controlMode::SPEED_CONTROL:
					drone->controller.setVYFIReference( -SPEED_COMMAND_SPEEDMODE);
					break;
				case Controller_MidLevel_controlMode::POSITION_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setYReference( yc - DELTA_POSITION_COMMAND_POSITIONMODE);
					break;
				case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
					break;
				}
			}
			break;
		case KEY_RIGHT:	// move right
			if (!drone->controller.isStarted())
				drone->setControlData(ControlChannel::PHI,   TILT_COMMAND_FREEMODE, 0.0f, 0.0f, 0.0f);
			else {
				switch (control_mode) {
				case Controller_MidLevel_controlMode::SPEED_CONTROL:
					drone->controller.setVYFIReference( +SPEED_COMMAND_SPEEDMODE);
					break;
				case Controller_MidLevel_controlMode::POSITION_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setYReference( yc + DELTA_POSITION_COMMAND_POSITIONMODE);
					break;
				case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
					break;
				}
			}
			break;
		case 's':		// set pitch,roll,dyaw,dz parrot direct commands to 0.0
			if (!drone->controller.isStarted())
				drone->setControlData(0.0f, 0.0f, 0.0f, 0.0f);
			else {
				switch (control_mode) {
				case Controller_MidLevel_controlMode::SPEED_CONTROL:
				case Controller_MidLevel_controlMode::POSITION_CONTROL:
					drone->controller.setAllReferences2ActualValues();
					break;
				case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
					drone->controller.setControlMode(MULTIROTOR_INIT_CONTROLMODE);
					drone->controller.setAllReferences2ActualValues();
					break;
				}
			}
			break;
		case 'a':		// go down
			if (!drone->controller.isStarted())
				drone->setControlData(ControlChannel::GAZ, 0.0f, 0.0f, -GAZ_COMMAND_FREEMODE, 0.0f);
			else {
				switch (control_mode) {
				case Controller_MidLevel_controlMode::SPEED_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setZReference( zc - DELTA_ALTITUDE_COMMAND_SPEEDMODE);
					break;
				case Controller_MidLevel_controlMode::POSITION_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setZReference( zc - DELTA_ALTITUDE_COMMAND_POSITIONMODE);
					break;
				case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
					break;
				}
			}
			break;
		case 'q':		// go up
			if (!drone->controller.isStarted())
				drone->setControlData(ControlChannel::GAZ, 0.0f, 0.0f,  GAZ_COMMAND_FREEMODE, 0.0f);
			else {
				switch (control_mode) {
				case Controller_MidLevel_controlMode::SPEED_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setZReference( zc + DELTA_ALTITUDE_COMMAND_SPEEDMODE);
					break;
				case Controller_MidLevel_controlMode::POSITION_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setZReference( zc + DELTA_ALTITUDE_COMMAND_POSITIONMODE);
					break;
				case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
					break;
				}
			}
			break;
		case 'z':		// turn leftwards
			if (!drone->controller.isStarted())
				drone->setControlData(ControlChannel::YAW, 0.0f, 0.0f, 0.0f, -YAW_COMMAND_FREEMODE);
			else {
				switch (control_mode) {
				case Controller_MidLevel_controlMode::SPEED_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setYawReference( yawc - DELTA_YAW_COMMAND_SPEEDMODE);
					break;
				case Controller_MidLevel_controlMode::POSITION_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setYawReference( yawc - DELTA_YAW_COMMAND_POSITIONMODE);
					break;
				case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setYawReference( yawc - DELTA_YAW_COMMAND_TRAJECTORYMODE);
					break;
				}
			}
			break;
		case 'x':		// turn rightwards
			if (!drone->controller.isStarted())
				drone->setControlData(ControlChannel::YAW, 0.0f, 0.0f, 0.0f,  YAW_COMMAND_FREEMODE);
			else {
				switch (control_mode) {
				case Controller_MidLevel_controlMode::SPEED_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setYawReference( yawc + DELTA_YAW_COMMAND_SPEEDMODE);
					break;
				case Controller_MidLevel_controlMode::POSITION_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setYawReference( yawc + DELTA_YAW_COMMAND_POSITIONMODE);
					break;
				case Controller_MidLevel_controlMode::TRAJECTORY_CONTROL:
					drone->controller.getCntReferences( &xc, &yc, &yawc, &zc);
					drone->controller.setYawReference( yawc + DELTA_YAW_COMMAND_TRAJECTORYMODE);
					break;
				}
			}
			break;
			/* END: ***** Movement commands ***** */

			/* ***** Multirotor parrot-like control modes ***** */
		case 'h':
			drone->setControlMode(HOVER);
			break;
		case 'm':
			drone->setControlMode(MOVE);
			break;
		case ' ':
			drone->setControlMode(EMERGENCYSTOP);
			break;
		case 'l':
			drone->setControlMode(LAND);
			break;
		case 't':
			drone->setControlMode(TAKEOFF);
			break;
			/* END: ***** Multirotor parrot-like control modes ***** */

			/* END: ***** Multirotor modules: stateObserver, Controller ***** */
		case 'k':
			drone->startStateObsever();
			break;
		case 'v':
			drone->setControlMode(MOVE);
			drone->controller.setControlMode(Controller_MidLevel_controlMode::SPEED_CONTROL);
			break;
		case 'b':
			drone->setControlMode(MOVE);
			drone->controller.setControlMode(Controller_MidLevel_controlMode::POSITION_CONTROL);
			break;
		case 'n': {
			drone->setControlMode(MOVE);
			drone->startStateObsever();
			//							usleep( 1e6);

			Controller_SM_TrajectoryConfiguration traj_config;
			traj_config.chk_R = 0.5;
			traj_config.chk_clearance_R = 0.5;

			cvg_double xs, ys, zs, yaws;
			drone->state_observer.getEstimatedQuadrotorPosition( xs, ys, zs, yaws);

			// Atlante sample code
			cvg_double L1 = 5.48;
			RotMatrix3 	M( Vector3( 0.0, 0.0, 1.0), yaws);
			Vector3 	ps( xs, ys, zs);
			Vector3	   	p0(  0.0,  0.0, 0.0);
			Vector3 	p1( L1,  0.0, 0.0);
			Vector3 	p2(  0.0,-L1, 0.0);
			Vector3 	p3( L1,-L1, 0.0);

			Vector3 	pi = ps + M*p0;
			Controller_SM_Trajectory_v2 trajectory( pi.x, pi.y, pi.z);
			pi = ps + M*p1;
			trajectory.addWaypoint( pi.x, pi.y, pi.z);
			pi = ps + M*p2;
			trajectory.addWaypoint( pi.x, pi.y, pi.z);
			pi = ps + M*p3;
			trajectory.addWaypoint( pi.x, pi.y, pi.z);
			trajectory.setPeriodic( true);
			trajectory.setInitialCheckpoint( 1);

			//							cvg_double xmi, ymi, zmi;
			//							xmi = 0.0; ymi = 0.0; zmi = 0.0;
			//							Controller_SM_Trajectory_v2 trajectory( xs + xmi*cos(yaws) - ymi*sin(yaws), ys + xmi*sin(yaws) + ymi*cos(yaws), zs + zmi);
			//							xmi = 5.48; ymi = 0.0; zmi = 0.0;
			//							trajectory.addWaypoint( xs + xmi*cos(yaws) - ymi*sin(yaws), ys + xmi*sin(yaws) + ymi*cos(yaws), zs + zmi);
			//							xmi = 0.0; ymi = -5.48; zmi = -1.0;
			//							trajectory.addWaypoint( xs + xmi*cos(yaws) - ymi*sin(yaws), ys + xmi*sin(yaws) + ymi*cos(yaws), zs + zmi);
			//							xmi = 5.48; ymi = -5.48; zmi = -1.0;
			//							trajectory.addWaypoint( xs + xmi*cos(yaws) - ymi*sin(yaws), ys + xmi*sin(yaws) + ymi*cos(yaws), zs + zmi);
			//							trajectory.setPeriodic( true);
			//							trajectory.setInitialCheckpoint( 1);

			//							// This is the sample code
			//							cvg_double z0 = -1.0;
			//							Controller_SM_Trajectory_v2 trajectory( xs + 0.0, ys + 0.0, z0);
			//							trajectory.addWaypoint( xs + 3.0,  ys + 0.0, z0 - 0.0);
			//							trajectory.addWaypoint( xs + 3.0,  ys + 3.0, z0 - 0.0);
			//							trajectory.addWaypoint( xs + 0.0,  ys + 3.0, z0 - 0.0);
			//							trajectory.setPeriodic( true);
			//							trajectory.setInitialCheckpoint( 1);

			drone->controller.setTrajectory( trajectory, traj_config);
			drone->controller.setControlMode(Controller_MidLevel_controlMode::TRAJECTORY_CONTROL);
			break;
		}
		// Start or stop video sources
/*

		case '0':
			drone->getChannelManager().enableVideoChannel(0, !drone->getChannelManager().isVideoChannelEnabled(0));
			break;
		case '1':
			drone->getChannelManager().enableVideoChannel(1, !drone->getChannelManager().isVideoChannelEnabled(1));
			break;
*/
			/* END: ***** Multirotor modules: EKF, Controller ***** */
		}
		usleep(100000);
	}
}

void ConsoleUI::writeLabel(cvg_uint y, cvg_uint x, const char *label, cvg_int valueLength, cvg_double value) {
	char str[valueLength + 1];
	snprintf(str, valueLength, "%.2f", value);
	mvprintw(y, x, "%s: %s", label, str);
	cvg_int l = strlen(str);
	cvg_int labelLen = strlen(label);
	for (cvg_int i = 0; i < valueLength - l; i++) mvprintw(y, x + labelLen + 2 + l + i, " ");
}

void ConsoleUI::refreshInfo() {
	char fmStr[32];
	flyingModeToString(fmStr, drone->getControlMode());
	char dmStr[64];
	droneModeToString(dmStr, lastDroneInfo.feedbackData.droneMode);
	sprintf(&dmStr[strlen(dmStr)], " (native: 0x%x)", lastDroneInfo.feedbackData.nativeDroneState);
	cvg_bool connToDrone = drone->connToDrone();	// This must be outside the lock-unlock to avoid potential deadlock
	if (consoleMutex.lock()) {
		int y = 0;
		mvprintw(y, 0, "Comm. with proxy:");
		if (drone->connToProxy()) {
			Channel::Statistics cStats = drone->getChannelManager().getControlChannel().getSendStatistics();
			Channel::Statistics fStats = drone->getChannelManager().getFeedbackChannel().getRecvStatistics();
			Channel::Statistics vStats = drone->getChannelManager().getVideoChannel(0).getRecvStatistics();
			mvprintw(y, 18, "OK (c:%.1f|f:%.1f|v:%.1f)     ", cStats.throughput, fStats.throughput, vStats.throughput);
		} else {
			mvprintw(y, 18, "ERROR                         ");
		}
		mvprintw(y++, 50, "Comm. with drone: %s   ", connToDrone ? "OK" : "ERROR");
		y++;
		mvprintw(y, 0, "Control mode: %s      ", fmStr);
		mvprintw(y, 30, "Drone mode: %s      ", connToDrone ? dmStr : "??");
		mvprintw(y += 2, 0, "Battery: %.2f%%  ", lastDroneInfo.feedbackData.batteryLevel * 100);
		y += 2;
		cvg_int backupY = y;
		// Odometry
		mvprintw(y++, 0, "ODOMETRY");
		writeLabel(y, 0, "x", 7, 0.0);
		writeLabel(y, 11, "y", 7, 0.0);
		writeLabel(y++, 22, "z", 7, -lastDroneInfo.feedbackData.altitude);
		writeLabel(y, 0, "Y", 7, lastDroneInfo.feedbackData.yaw);
		writeLabel(y, 11, "P", 7, lastDroneInfo.feedbackData.pitch);
		writeLabel(y++, 22, "R", 7, lastDroneInfo.feedbackData.roll);
		writeLabel(y, 0, "Vf", 7, lastDroneInfo.feedbackData.speedX);
		writeLabel(y++, 11, "Vl", 7, lastDroneInfo.feedbackData.speedY);
		//		mvprintw(y++, 25, "Vyaw: %.2f", lastDroneInfo.feedbackData.speedYaw);

		// Vicon
		y = backupY;
		MyDrone::ViconData vd;
		if (drone->getViconData(&vd)) {
			mvprintw(y++, 35, "VICON");
			Vector3 pos = vd.localTransform.getTranslation() * 1e-3;
			writeLabel(y, 35, "x", 7, pos.x);
			writeLabel(y, 46, "y", 7, pos.y);
			writeLabel(y++, 57, "z", 7, pos.z);
			Vector3 euler = vd.localTransform.getRotationMatrix().getEulerAnglesZYX();
			writeLabel(y, 35, "Y", 7, euler.z * 180.0 / M_PI);
			writeLabel(y, 46, "P", 7, euler.y * 180.0 / M_PI);
			writeLabel(y++, 57, "R", 7, euler.x * 180.0 / M_PI);
			writeLabel(y, 35, "Vf", 7, vd.localSpeed.x * 1e-3);
			writeLabel(y, 46, "Vl", 7, vd.localSpeed.y * 1e-3);
			//			mvprintw(y++, 25, "Vyaw: %.2f", lastDroneInfo.feedbackData.speedYaw);
		}

		refresh();
		consoleMutex.unlock();
	}
}

void ConsoleUI::logConsume(DroneProxy::Logs::LogProducer *producer, const cvgString &str) {
	if (consoleMutex.lock()) {
		if (!isInit) {
			printf("%s\n", str.c_str());
		} else {
			// Break string if it's wider than the window
			int maxx, maxy;
			getmaxyx(logWin, maxy, maxx);
			maxx = ((maxx - 2) - 1) + 1;	// Substract border
			cvg_int numFrags = (cvg_int)ceil(str.length() / (cvg_double)maxx);
			//		if (consoleMutex.lock()) {
			for (cvg_int i = 0; i < numFrags - 1; i++) {
				wprintw(logWin, (" " + str.subString(i * maxx, maxx) + "\n").c_str());
			}
			wprintw(logWin, (" " + str.subString((numFrags - 1) * maxx) + "\n").c_str());
			//			consoleMutex.unlock();
			//		}
			box(logWin, 0, 0);
			wrefresh(logWin);

		}
		consoleMutex.unlock();
	}
}

void ConsoleUI::gotData(Channel *channel, void *data) {
	if (channel == &drone->getChannelManager().getFeedbackChannel()) {
		if (consoleMutex.lock()) {
			memcpy(&lastDroneInfo.feedbackData, data, sizeof(lastDroneInfo.feedbackData));
			consoleMutex.unlock();
		}
	}
}
