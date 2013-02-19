/*
 * Controller_MidLevel_controlModes.h
 *
 *  Created on: Nov 26, 2012
 *      Author: jespestana
 */

#ifndef CONTROLLER_MIDLEVEL_CONTROLMODES_H_
#define CONTROLLER_MIDLEVEL_CONTROLMODES_H_

// Control mode names declaration
namespace Controller_MidLevel_controlMode {
	enum controlMode {
		TRAJECTORY_CONTROL = 1,
		POSITION_CONTROL,
		SPEED_CONTROL
	};
}


#endif /* CONTROLLER_MIDLEVEL_CONTROLMODES_H_ */
