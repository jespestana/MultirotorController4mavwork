/*
 * FilteredDerivative.h
 *
 *  Created on: Dec 14, 2012
 *      Author: jespestana
 */

#ifndef FILTEREDDERIVATIVE_H_
#define FILTEREDDERIVATIVE_H_

#include <atlante.h>
#include "Other/Timer.h"
#include "controller/other/LowPassFilter.h"

namespace CVG_BlockDiagram {

class FilteredDerivative {
private:
	// Input/output
	cvg_double x_k; // input
	cvg_double dx_k; // output

	// Parameters
	LowPassFilter lowpassfilter;
	Timer timer;

	// Internal state
	cvg_double x_km1; // last output
	cvg_bool started;

public:
	FilteredDerivative();
	virtual ~FilteredDerivative();

	void reset();

	inline void setResponseTime(cvg_double tr) { lowpassfilter.setResponseTime(tr); }
	void enableSaturation(cvg_bool enable, cvg_double min, cvg_double max);

	inline void setInternaldxk(cvg_double dx_kint) { lowpassfilter.setInternalyk( dx_kint); }
	inline void setInput(cvg_double x_k) 	{ this->x_k = x_k; }
	inline cvg_double getInput() 			{ return x_k; }
	cvg_double getOutput(bool use_last_input = false);
};

}

#endif /* FILTEREDDERIVATIVE_H_ */
