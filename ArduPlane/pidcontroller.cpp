#include "pidcontroller.h"

PIDController::PIDController()
{
	this->Kp = 0;
	this->Ki = 0;
	this->Kd = 0;
	this->ilimit = 1000;
	this->integral = 0;
	this->prev_error = 0;
}

PIDController::PIDController(float _Kp, float _Ki, float _Kd)
{
	this->Kp = _Kp;
	this->Ki = _Ki;
	this->Kd = _Kd;
	this->ilimit = 1000;
	this->integral = 0;
	this->prev_error = 0;
}

PIDController::PIDController(float _Kp, float _Ki, float _Kd, float _ilim)
{
	this->Kp = _Kp;
	this->Ki = _Ki;
	this->Kd = _Kd;
	this->ilimit = _ilim;
	this->integral = 0;
	this->prev_error = 0;
}

void PIDController::setGains(float _Kp, float _Ki, float _Kd)
{
	this->Kp = _Kp;
	this->Ki = _Ki;
	this->Kd = _Kd;
}

void PIDController::setIntegralLimit(float _ilimit)
{
	this->ilimit = _ilimit;
}

float PIDController::run(float dt, float error)
{
	this->integral += error*dt;
	this->integral = saturate(this->integral, -this->ilimit, this->ilimit);
	
	float output;
	
	output = this->Kp*error;
	output += this->Ki*this->integral;
	output += this->Kd*((error - this->prev_error)/dt);
	
	this->prev_error = error;
	
	return output;
}

float PIDController::saturate(float val, float min, float max)
{
	if (val > max) {
		return max;
	} else if (val < min) {
		return min;
	} else {
		return val;
	}
}

float PIDController::wrap(float val, float min, float max)
{
	while (val > max) {
		val += (min - max);
	}
	while (val < min) {
		val += (max - min);
	}
	
	return val;
}
