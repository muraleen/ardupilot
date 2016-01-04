#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <DataFlash/DataFlash.h>

class PIDController
{
	public:
		PIDController(float _Kp, float _Ki, float _Kd);
		void setGains(float _Kp, float _Ki, float _Kd);
		void setIntegralLimit(float _ilimit);
		void resetIntegrator();
		float run(float dt, float error);
		
		static float saturate(float val, float min, float max);
		static float wrap(float val, float min, float max);

		const DataFlash_Class::PID_Info& get_pid_info(void) const { return _pid_info; }
	
	private:
		float integral;
		float prev_error;
		float Kp;
		float Ki;
		float Kd;
		float ilimit;
		DataFlash_Class::PID_Info _pid_info;
};

#endif
