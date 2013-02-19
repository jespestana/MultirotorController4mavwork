/*
 * stringdatalogger.cpp
 *
 *  Created on: Dec, 2012
 *      Author: jespestana
 */

#include "Other/stringdatalogger.h"

Stringdatalogger::Stringdatalogger(const char* fname) {
	filename = fname;
	myfile.clear();
	myfile.open(fname,  std::ofstream::out | std::ofstream::in | std::ofstream::trunc);

	if (!myfile.is_open()) {
		printf("file: %s, could not be opened correctly.\n", fname);
		exit(1);
	}
}

Stringdatalogger::~Stringdatalogger() {
//	myfile.flush();
	myfile.close();
}

void Stringdatalogger::logdata(cvgString& log_msg) {
	myfile << log_msg << std::endl;
}
