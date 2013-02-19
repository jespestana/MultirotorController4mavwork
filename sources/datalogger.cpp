/*
 * datalogger.cpp
 *
 *  Created on: Apr 25, 2012
 *      Author: jespestana
 */

#include "Other/datalogger.h"

	datalogger::datalogger(const char * timename, const char* fname, const char* var_names) {
		filename = fname;
		myfile.clear();
		myfile.open(fname,  std::ofstream::out | std::ofstream::in | std::ofstream::trunc);

		if (!myfile.is_open()) {
			printf("file: %s, could not be opened correctly.\n", fname);
			exit(1);
		}

		myfile << timename << " " << var_names << "\n";

		varNames = var_names;
		timeName = timename;
		n = 0;
	}
	datalogger::~datalogger() {
//		myfile.flush();
		myfile.close();
	}

void datalogger::restart() {

	n += 1;
	size_t place = filename.find(".txt");
	filename.insert(place, "A" );

	myfile.clear();
	myfile.open(filename.c_str(),  std::ofstream::out | std::ofstream::in | std::ofstream::trunc);

	if (!myfile.is_open()) {
		std::cout << "file: %s, could not be opened correctly.\n" << filename;
		exit(1);
	}

	myfile << timeName << " " << varNames << "\n";

}

void datalogger::logdata(double time, Vector *dataVector) {

	myfile << time << " ";
		for (int i=1; i<=dataVector->length(); i++) {
			myfile << dataVector->getValueData(i) << " ";
		}
		myfile << "\n";

	}
