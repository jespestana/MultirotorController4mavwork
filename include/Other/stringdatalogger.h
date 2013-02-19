/*
 * stringdatalogger.h
 *
 *  Created on: Dec, 2012
 *      Author: jespestana
 */

#ifndef STRINGDATALOGGER_H_
#define STRINGDATALOGGER_H_

// basic file operations
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <atlante.h>


class Stringdatalogger {

private:
	std::ofstream myfile;
	std::string   filename;

public:
	Stringdatalogger(const char* filename);
	~Stringdatalogger();
	void logdata(cvgString& log_msg);
};


#endif /* STRINGDATALOGGER_H_ */
