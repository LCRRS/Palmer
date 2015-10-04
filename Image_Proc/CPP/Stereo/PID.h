#ifndef PID_H
#define PID_H

#include <iostream>
#include <cmath>

using namespace std;

class PID{
	public:
		PID(float*,float*,float*,float,float,float);		// constructor

		void Compute();
		void SetLimits(float*,float*);


	private:
		float *input;              // * Pointers to the Input, Output, and Setpoint variables
    	float *output;             //   This creates a hard link between the variables and the
    	float *setpoint;           //   PID, freeing the user from having to constantly tell us

    	float kp;
    	float ki;
    	float kd;

    	float *upper;
    	float *lower;

};

#endif