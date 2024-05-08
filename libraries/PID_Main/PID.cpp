#include "Arduino.h"
#include "PID.h"
#include "math.h"

PIDClass::PIDClass()
{
	Kp = 1;
	Ki = 1;
	Kd = 1;

	currentTime = 0;
	current = 0;
	integralTerm = 0;
}

void PIDClass::kConstants(float p, float i, float d)
{
	Kp = p;
	Ki = i;
	Kd = d;
}

float PIDClass::proportionalFun(float Kp, float error)
{
	return (Kp * error);
}

float PIDClass::integralFun(float Ki, float error, float elapsed)
{
	return (Ki * (error * elapsed));
}

float PIDClass::derivativeFun(float Kd, float current, float last, float elapsed)
{
	return -(Kd * ((current - last) / elapsed));
}

float PIDClass::elapsedTimeFun()
{
	double pastTime = currentTime;
	currentTime = millis();
	return (currentTime - pastTime);
}

float PIDClass::PID(float target, float CP, float upperLimit, float lowerLimit, float intUpperLimit, float intLowerLimit, bool circle)
{
	float last = current;
	current = CP;
	
	float error;
	float lastError = error;
	if (circle == true)
		error = abs(fmod((target - current + 180), 360)) - 180;
	else
		error = (target - current);

	//get the elapsed time since the last update
	float elapsedTime = elapsedTimeFun();

	//gets the proportional term value
	float proportionalTerm = proportionalFun(Kp, error);

	integralTerm += integralFun(Ki, error, elapsedTime);
	//check if integral term is within limits to prevent windup
	if (integralTerm > intUpperLimit)
		integralTerm = intUpperLimit;
	else if (integralTerm < intLowerLimit)
		integralTerm = intLowerLimit;
	//gets the derivative term value
	float derivativeTerm = derivativeFun(Kd, current, last, elapsedTime);

	//adds all terms together
	float outputTerm = (proportionalTerm + integralTerm + derivativeTerm);
	//checks if the final output is within the limits and corrects if not
	if (outputTerm > upperLimit)
		outputTerm = upperLimit;
	else if (outputTerm < lowerLimit)
		outputTerm = lowerLimit;

	//returns the final value
	return outputTerm;
}

void PIDClass::resetPID(void)
{
	currentTime = millis();
	current = 0;
	integralTerm = 0;
}