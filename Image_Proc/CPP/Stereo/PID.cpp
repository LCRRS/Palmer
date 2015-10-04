#include "PID.h"

using namespace std;

PID::PID(float* xInput, float* xOutput, float* xSetpoint, float Kp, float Ki, float Kd){
	input = xInput;
	output = xOutput;
	setpoint = xSetpoint;

	kp = Kp;
	ki = Ki;
	kd = Kd;
}

void PID::Compute(){
	float new_input = *input;
	float error = *setpoint - new_input;
	float new_output = (error * kp);
	if(new_output > *upper){
		*output = *upper;
	}
	else if(new_output < *lower){
		*output = *lower;
	}
	else{
		*output = new_output;
	}
	return;
}

void PID::SetLimits(float* xUpper, float* xLower){
	upper = xUpper;
	lower = xLower;
}

