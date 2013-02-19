/*
 * main.cpp
 *
 *  Created on: 12/08/2011
 *      Author: Ignacio Mellado
 *  Modified  : 31/10/2012
 *      Author: Jesus Pestana
 */

#include <ConsoleUI.h>
#include <stdio.h>
#include <iostream>
#include <atlante.h>
//#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <MyDrone.h>
#include <droneproxy.h>

using namespace DroneProxy;

int main(void) {
	MyDrone myDrone;
	ConsoleUI ui(&myDrone);
	
	try {

//		myDrone.setAuto(false);
//		myDrone.setPositioningSource(MyDrone::POSITIONING_VICON);
		// PELICAN_HOST "10.0.100.3"
		// DRONE_HOST	"127.0.0.1"
		myDrone.open(cvgString(MULTIROTOR_PROXY_HOST_IP), COMMANDER,
				0, cvgString("arcaa-vicon-server:801"), cvgString("Nacho-ARDrone2"));
		myDrone.logDataFrom(MyDrone::ODOMETRY | MyDrone::COMMANDS); // | MyDrone::VIDEO  // | MyDrone::VICON);

		ui.init();
		ui.doLoop();
		ui.destroy();

		myDrone.close();
	} catch(cvgException &e) {
		ui.destroy();
		myDrone.close();
		fprintf(stderr, "[Program exception] %s\n", e.getMessage().c_str());
	}

	return 0;
}




