/**
* @file pid.hpp
* @brief PID regulator class
*/
#ifndef PID_HPP
#define PID_HPP

#include <stdint.h>


class PidRegulator
{
public:
	typedef float Input_t;
	typedef int8_t Output_t;
	typedef const float Coefficient_t;

	PidRegulator(Coefficient_t p, Coefficient_t i, Coefficient_t d, float t);
	void SetValue(Input_t desiredInput);
	Output_t Do(Input_t currentInput);

private:
	typedef float Integral_t;
	typedef float Error_t;

	Coefficient_t K_P;
	Coefficient_t K_I;
	Coefficient_t K_D;
	const float DeltaT;
	Integral_t Integral;
	Input_t DesiredInput;
	const Output_t OUTPUT_MAX;
	const Output_t OUTPUT_MIN;
};

#endif /* PID_HPP */
