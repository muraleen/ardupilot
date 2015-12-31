#include "deepstall.h"
#include <math.h>

#define M_PI 3.14159265359

DeepStall::DeepStall() {
	YawRateController = new YawRateController(0,0,0);
	YawRateController->setIntegralLimit(0);
	land_lat = 0;
	land_lon = 0;
	Kyr = 0;
	rCmd = 0;
	eCmd = 0;
	targetHeading = 0;	
}

void DeepStall::setTarget(double lat, double lon) {
	double land_lat = lat;
	double land_lon = lon;
}

void DeepStall::setYRCParams(double _Kyr, double _yrLimit, double Kp, double Ki, double Kd, double ilim) {
	YawRateController->setGains(Kp, Ki, Kd);
	YawRateController->setIntegralLimit(ilim);
	Kyr = _Kyr;
	yrLimit = _yrLimit;
}

void DeepStall::compute(double dt, double track, double yawrate, double lat, double lon) {
	// double targetTrack = atan2(land_lat-lat, land_lon-lon);
	double targetTrack = targetHeading*M_PI/180;
	track *= M_PI/180; // Convert to radians
	
	// For testing purposes, a heading hold is used instead of a track hold - the track variable is actually heading
	
	rCmd = saturate(YawRateController->run(dt, saturate(Kyr*wrap(targetTrack - track, -M_PI, M_PI), -yrLimit, yrLimit)), -1, 1);
}

double DeepStall::getRudderNorm() {
	return rCmd;
}

double DeepStall::getElevatorNorm() {
	return eCmd;
}

void DeepStall::setTargetHeading(double hdg) {
	targetHeading = hdg;
}
