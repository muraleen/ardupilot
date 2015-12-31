#include "pidcontroller.h"

PIDController::PIDController(double _Kp, double _Ki, double _Kd)
{
	this->Kp = _Kp;
	this->Ki = _Ki;
	this->Kd = _Kd;
	this->ilimt = 1000;
	this->integral = 0;
	this->prev_error = 0;
}

double PIDController::setGains(double _Kp, double _Ki, double _Kd)
{
	this->Kp = _Kp;
	this->Ki = _Ki;
	this->Kd = _Kd;
}

void PIDController::setIntegralLimit(double _ilimit)
{
	this->ilimit = _ilimit;
}

double PIDController::run(double dt, double error)
{
	this->integral += error*dt;
	this->integral = saturate(this->integral, -this->ilimit, this->ilimit);
	
	double output;
	
	output = this->Kp*error;
	output += this->Ki*this->integral;
	output += this->Kd*((error - this->prev_error)/dt);
	
	this->prev_error = error;
	
	return output;
}

double PIDController::saturate(double val, double min, double max)
{
	if (val > max) {
		return max;
	} else if (val < min) {
		return min;
	} else {
		return val;
	}
}

double PIDController::wrap(double val, double min, double max)
{
	while (val > max) {
		val += (min - max);
	}
	while (val < min) {
		val += (max - min);
	}
	
	return val;
}
