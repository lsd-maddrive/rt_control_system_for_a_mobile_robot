/**
* @file pid.cpp
* @brief PID class implementation
*/

#include "pid.hpp"


PidRegulator::PidRegulator(Coefficient_t p, Coefficient_t i, Coefficient_t d, float deltaT):
	K_P(p), K_I(i), K_D(d), DeltaT(deltaT), Integral(0), DesiredInput(0),
	OUTPUT_MAX(100), OUTPUT_MIN(-100) {}


void PidRegulator::SetValue(Input_t desiredInput)
{
	DesiredInput = desiredInput;
}


PidRegulator::Output_t PidRegulator::Do(PidRegulator::Input_t currentInput)
{
	// Calculate error
	Input_t error = (DesiredInput - currentInput);

	// Proportional part:
	Error_t pError = K_P * error;

	// Integral part:
	Integral += K_I * error * DeltaT;
	if(Integral > OUTPUT_MAX)
		Integral = OUTPUT_MAX;
	else if(Integral < OUTPUT_MIN)
		Integral = OUTPUT_MIN;
	Error_t iError = Integral;

	// Derivative part:
	Error_t dError = 0;

	// Check boundary and return:
	Error_t errorSum = pError + iError + dError;
	if(errorSum > OUTPUT_MAX)
		return OUTPUT_MAX + 1;
	else if(errorSum < OUTPUT_MIN)
		return OUTPUT_MIN - 1;
	return Output_t(errorSum);
}
