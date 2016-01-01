#include "deepstall.h"
#include <math.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// #define M_PI 3.14159265359

DeepStall::DeepStall() {
	YawRateController = new PIDController(0,0,0);
	YawRateController->setIntegralLimit(0);
	land_lat = 0;
	land_lon = 0;
	Kyr = 0;
	rCmd = 0;
	eCmd = 0;
	targetHeading = 0;
	_last_t = 0;
}

void DeepStall::setTarget(float lat, float lon) {
	land_lat = lat;
	land_lon = lon;
}

void DeepStall::setYRCParams(float _Kyr, float _yrLimit, float Kp, float Ki, float Kd, float ilim) {
	YawRateController->setGains(Kp, Ki, Kd);
	YawRateController->setIntegralLimit(ilim);
	Kyr = _Kyr;
	yrLimit = _yrLimit;
}

void DeepStall::compute(float track, float yawrate, float lat, float lon) {

	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 10; // Default to 100 Hz
	}
	_last_t = tnow;

	// float targetTrack = atan2(land_lat-lat, land_lon-lon);
	float targetTrack = targetHeading*M_PI/180;
	// track *= M_PI/180; // Convert to radians (seems to already be in radians)
	
	// For testing purposes, a heading hold is used instead of a track hold - the track variable is actually heading
	
	rCmd = PIDController::saturate(YawRateController->run(((float) dt)/1000.0, PIDController::saturate(Kyr*PIDController::wrap(targetTrack - track, -M_PI, M_PI), -yrLimit, yrLimit)), -1, 1);
	
	eCmd = 1; // Full pitch up
}

float DeepStall::getRudderNorm() {
	return rCmd;
}

float DeepStall::getElevatorNorm() {
	return eCmd;
}

void DeepStall::setTargetHeading(float hdg) {
	targetHeading = hdg;
}
