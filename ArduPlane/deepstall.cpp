#include "deepstall.h"
#include <math.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// #define M_PI 3.14159265359

DeepStall::DeepStall() {
	YawRateController = new PIDController(0,0,0);
	YawRateController->setIntegralLimit(0);
	TargetPositionController = new PIDController(0,0,0);
	TargetPositionController->setIntegralLimit(0);
	land_lat = 0;
	land_lon = 0;
	Kyr = 0;
	rCmd = 0;
	targetHeading = 0;
	_last_t = 0;
	d_predict = 0;
	stage = FLY_TO_ARC;
	ready = false;
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

void DeepStall::setTPCParams(float Kp, float Ki, float Kd, float _ilim) {
	TargetPositionController->setGains(Kp, Ki, Kd);
	TargetPositionController->setIntegralLimit(_ilim);
}

void DeepStall::computeApproachPath(Vector3f _wind, float loiterRadius, float d_s, float v_d, float deltah, float vspeed, float lat, float lon) {

	float course = targetHeading*M_PI/180;

	// Generate v_d and wind vectors
	Vector3f Vd(v_d*sin(course), v_d*cos(course), 0);
	Vector3f wind(_wind.y, _wind.x, 0);
	
	// Compute effective groundspeed - can be negative, hence can handle backward tracking
	float v_e = (1/v_d)*(Vd * (Vd + wind)); // should essentially do dot(Vd,Vd+wind)/v_d
	
	// Predict deepstall distance (can handle backward tracking! xD)
	d_predict = v_e*deltah/vspeed;
	
	// Compute deepstall entry waypoint
	lat_e = land_lat + d_predict*cos(course + M_PI)/59.71/1852;
	lon_e = land_lon + d_predict*sin(course + M_PI)/59.71/1852;
	
	// Compute course intercept waypoint
	lat_i = lat_e + d_s*cos(course + M_PI)/59.71/1852;
	lon_i = lon_e + d_s*sin(course + M_PI)/59.71/1852;
	
	/* LOITER AND INTERCEPT (not being used at the moment)
	// Compute pre-final loiter center
	float dangle = 3*M_PI/2;
	if (PIDController::wrap(course - atan2(land_lon-lon, land_lat-lat), -M_PI, M_PI) > 0) {
		dangle = M_PI/2;
		loiter_ccw = false; // So, loiter clockwise
	}
	
	lat_l = lat_i + loiterRadius*cos(course + dangle)/59.71/1852;
	lon_l = lon_i + loiterRadius*sin(course + dangle)/59.71/1852;
	*/
	
	// ARC INTERSECTION AND INTERCEPT (active)
	float iangle = atan2(lon_i-lon, lat_i-lat);
	
	if (PIDController::wrap(course - iangle, -M_PI, M_PI) < -M_PI/4) {
		// Left side of the arc
		lat_l = lat_i + loiterRadius*cos(course + 5*M_PI/4)/59.71/1852;
		lon_l = lon_i + loiterRadius*sin(course + 5*M_PI/4)/59.71/1852;
	} else if (PIDController::wrap(course - iangle, -M_PI, M_PI) > M_PI/4) {
		// Right side of the arc
		lat_l = lat_i + loiterRadius*cos(course + 3*M_PI/4)/59.71/1852;
		lon_l = lon_i + loiterRadius*sin(course + 3*M_PI/4)/59.71/1852;
	} else {
		// Intersects with the arc
		lat_l = lat_i + loiterRadius*cos(iangle + M_PI)/59.71/1852;
		lon_l = lon_i + loiterRadius*sin(iangle + M_PI)/59.71/1852;
	}
	
	hal.console->printf("Stage0: %3.8f %3.8f\n", lat_l, lon_l);
	hal.console->printf("Stage1: %3.8f %3.8f\n", lat_i, lon_i);
	hal.console->printf("Stage2: %3.8f %3.8f\n", lat_e, lon_e);
	hal.console->printf("Wind Heading: %3.0f\n\n", targetHeading);
	
	// hal.console->printf("%3.2f \t %3.7f \t %3.7f \t\t %3.7f \t %3.7f \t\t %3.7f \t %3.7f \n", iangle*180/M_PI, lat_l, lon_l, lat_i, lon_i, lat_l, lon_l);
	
	// DONE! The three approach waypoints (until switching to the compute function below) are stored in the class. setApproachPath() should copy those into the flight plan and execute the approach
	
}

bool DeepStall::getApproachWaypoint(Location &target, Location &land_loc, Location &current, Vector3f _wind, float v_d, float deltah, float vspeed) {
	
	float course = targetHeading*M_PI/180;
	
	// Generate v_d and wind vectors
	Vector3f Vd(v_d*sin(course), v_d*cos(course), 0);
	Vector3f wind(_wind.y, _wind.x, 0);
	
	// Compute effective groundspeed - can be negative, hence can handle backward tracking
	float v_e = (1/v_d)*(Vd * (Vd + wind)); // should essentially do dot(Vd,Vd+wind)/v_d
	
	// Predict deepstall distance (can handle backward tracking! xD)
	d_predict = v_e*deltah/vspeed;
	
	float tgt_lat = 0, tgt_lon = 0;
	
	switch (stage) {
		case FLY_TO_ARC: // Fly-to entry arc point (lat_l, lon_l)
			tgt_lat = lat_l;
			tgt_lon = lon_l;
			break;
		case COURSE_INTERCEPT: // Fly-to course intercept point (lat_i, lon_i)
			tgt_lat = lat_i;
			tgt_lon = lon_i;
			break;
		case DEEPSTALL_ENTRY: // Fly-to deepstall entry point (lat_e, lon_e)
			tgt_lat = lat_e;
			tgt_lon = lon_e;
			break;
		case DEEPSTALL_LAND: // Land
			return false;
	}
	
	target.lat = (int32_t) (tgt_lat * 1.0e7f);
	target.lng = (int32_t) (tgt_lon * 1.0e7f);
	
	if ((get_distance(current, land_loc) <= d_predict + 5 && stage==DEEPSTALL_ENTRY) || (get_distance(current, target) < 25 && stage<DEEPSTALL_ENTRY)) {
		stage++;
		hal.console->printf("Deepstall stage: %d\n", stage);
	}
	
	return true;
}

void DeepStall::abort() {
	YawRateController->resetIntegrator();
    TargetPositionController->resetIntegrator();
    stage = FLY_TO_ARC; // Reset deepstall stage in case of abort
    ready = false;
}

void DeepStall::land(float track, float yawrate, float lat, float lon) {

	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 10; // Default to 100 Hz
	}
	_last_t = tnow;

	if (lat == 0 || lon == 0) {	
		targetTrack = targetHeading*M_PI/180;
	} else {
		// targetTrack = atan2(land_lon-lon, land_lat-lat);
		
		// Target position controller
		// Generate equation of the tracking line parameters
		float course = targetHeading*M_PI/180;
		
		float x1 = 1;
		float y1 = 1;
		float x2 = (x1 + sin(course));
		float y2 = (y1 + cos(course));
		
		float x0 = (lon-land_lon+1);
		float y0 = (lat-land_lat+1);
		
		// Distance between a point and a line: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points [doesn't work!!], I derived the equation below from basic geometry and that seems to work fine.
		// float d2trk = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)/sqrt(pow(y2-y1,2) + pow(x2-x1,2))*1852;
		
		// float m = (y2-y1)/(x2-x1);
		//float d2trk = sqrt(pow(y0-y1-m*x0+m*x1,2)/(pow(m,2)+1))*1852;
		
		float d2trk = sqrt(pow(x0*y1 - x1*y0 - x0*y2 + x2*y0 + x1*y2 - x2*y1,2)/(pow(x1-x2,2)+pow(y1-y2,2)))*1852*59.71;
		// float d2trk = sqrt(pow((lon-land_lon)*cos(course) + (lat-land_lat)*sin(course),2)/(pow(sin(course),2)+pow(cos(course),2)))*1852*59.71;
		
		// bool turn_right = true;
		// Positive d2trk means turn right :)
		if (PIDController::wrap(course - atan2(land_lon-lon, land_lat-lat), -M_PI, M_PI) > 0) {
			// turn_right = false; // So basically, turn left
			d2trk *= -1; // And negative d2trk means turns left
		}
		
		targetTrack = PIDController::wrap(course + PIDController::saturate(TargetPositionController->run(((float) dt)/1000.0, d2trk),-M_PI/4,M_PI/4),0,2*M_PI);
		
		// hal.console->printf("%2.4f \t %3.2f \t %3.2f \n", atan2(land_lon-lon, land_lat-lat), d2trk, targetTrack*180/M_PI);
	}
	
	rCmd = PIDController::saturate(YawRateController->run(((float) dt)/1000.0, PIDController::saturate(Kyr*PIDController::wrap(targetTrack - track, -M_PI, M_PI), -yrLimit, yrLimit)), -1, 1);
}

float DeepStall::getRudderNorm() {
	return rCmd;
}

void DeepStall::setTargetHeading(float hdg) {
	targetHeading = hdg;
}
