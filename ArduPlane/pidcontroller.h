#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController
{
	public:
		PIDController(float _Kp, float _Ki, float _Kd);
		void setGains(float _Kp, float _Ki, float _Kd);
		void setIntegralLimit(float _ilimit);
		float run(float dt, float error);
		
		static float saturate(float val, float min, float max);
		static float wrap(float val, float min, float max);
	
	private:
		float integral;
		float prev_error;
		float Kp;
		float Ki;
		float Kd;
		float ilimit;
};

#endif
