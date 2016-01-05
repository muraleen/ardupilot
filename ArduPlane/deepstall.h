#ifndef DEEPSTALL_H
#define DEEPSTALL_H

#include "pidcontroller.h"
#include <stdint.h>
#include <AP_Math/AP_Math.h>

class DeepStall
{
	public:
		DeepStall();
		void setTarget(float lat, float lon);
		void setYRCParams(float _Kyr, float _yrLimit, float Kp, float Ki, float Kd, float ilim);
		void setTPCParams(float Kp, float Ki, float Kd, float _ilim);
		void compute(float track, float yawrate, float lat, float lon);
		float getRudderNorm();
		float getElevatorNorm();
		
		void computeApproachPath(Vector3f wind, float loiterRadius, float d_s, float v_d, float deltah, float vspeed, float lat, float lon);
		void setApproachPath();
		
		void setTargetHeading(float hdg);
		PIDController *YawRateController;
		PIDController *TargetPositionController;
	
	private:
		float land_lat;
		float land_lon;
		float rCmd;
		float eCmd;
		float targetHeading;
		float Kyr;
		float yrLimit;
		uint32_t _last_t;
		
		// Approach parameters
		float d_predict;
		float v_e;
		// Deepstall entry point
		float lat_e;
		float lon_e;
		// Course intercept point
		float lat_i;
		float lon_i;
		// Pre-final loiter
		float lat_l;
		float lon_l;
		
};

#endif
