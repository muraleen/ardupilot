#ifndef DEEPSTALL_H
#define DEEPSTALL_H

#include "pidcontroller.h"

// Frequency domain analysis suggests that the PID gains should be: 240, 80 0
// Try a Kyr gain of between 2 and 4 - tune accordingly.
// Kyr should be close to 1/T where T is the unsaturated settling time

// Possible parameters?
// DPSTL_KP		Proportional gain for yaw rate PID controller
// DPSTL_KI		Integral gain for ...
// DPSTL_KD		Derivative gain for ...
// DPSTL_KYR	Yaw rate multiplier for heading/track control
// DPSTL_YRLIM	Yaw rate limit
// DPSTL_ILIM	Integral limit for yaw rate PID controller

class DeepStall
{
	public:
		DeepStall();
		void setTarget(double lat, double lon);
		void setYRCParams(double _Kyr, double _yrLimit, double Kp, double Ki, double Kd, double ilim);
		void compute(double dt, double track, double yawrate, double lat, double, lon);
		double getRudderNorm();
		double getElevatorNorm();
		
		void setTargetHeading(double hdg);
	
	private:
		PIDController *YawRateController;
		double land_lat;
		double land_lon;
		double rCmd;
		double eCmd;
		double targetHeading;
		double Kyr;
		double yrLimit;
};

#endif
