#ifndef DEEPSTALL_H
#define DEEPSTALL_H

#include "pidcontroller.h"
#include <stdint.h>

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
		void setTarget(float lat, float lon);
		void setYRCParams(float _Kyr, float _yrLimit, float Kp, float Ki, float Kd, float ilim);
		void compute(float track, float yawrate, float lat, float lon);
		float getRudderNorm();
		float getElevatorNorm();
		
		void setTargetHeading(float hdg);
		PIDController *YawRateController;
	
	private:
		float land_lat;
		float land_lon;
		float rCmd;
		float eCmd;
		float targetHeading;
		float Kyr;
		float yrLimit;
		uint32_t _last_t;
};

#endif
