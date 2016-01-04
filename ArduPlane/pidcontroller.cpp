#include "pidcontroller.h"

PIDController::PIDController(float _Kp, float _Ki, float _Kd)
{
	this->Kp = _Kp;
	this->Ki = _Ki;
	this->Kd = _Kd;
	this->ilimit = 1000;
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

void PIDController::resetIntegrator()
{
	this->integral = 0;
}

float PIDController::run(float dt, float error)
{
	_pid_info.desired = error;

	this->integral += error*dt;
	this->integral = saturate(this->integral, -this->ilimit, this->ilimit);
	
	_pid_info.P = this->Kp*error;
	_pid_info.I = this->Ki*this->integral;
	_pid_info.D = this->Kd*((error - this->prev_error)/dt);
	
	float output = _pid_info.P + _pid_info.I + _pid_info.D;
	
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
