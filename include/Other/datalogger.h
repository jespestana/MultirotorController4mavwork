/*
 * datalogger.h
 *
 *  Created on: Apr 25, 2012
 *      Author: jespestana
 */

// I am not using this code right now (17-12-2012), but I wanted to have the source code at hand just in case.

#ifndef DATALOGGER_H_
#define DATALOGGER_H_

// basic file operations
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include "stateObserver/EKF_lib/matrixLib.h"
#include "Other/jesus_library.h"


class datalogger {

private:
	std::ofstream myfile;
	std::string   filename;
	std::string   varNames;
	std::string   timeName;
	int n;

public:
	datalogger(const char* timename, const char* filename, const char* var_names);
	~datalogger();
	void logdata(double time, Vector *dataVector);
	void restart();

};


#endif /* DATALOGGER_H_ */
