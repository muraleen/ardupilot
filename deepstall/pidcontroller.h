#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController
{
	public:
		PIDController(double _Kp, double _Ki, double _Kd);
		void setGains(double _Kp, double _Ki, double _Kd);
		void setIntegralLimit(double _ilimit);
		double run(double dt, double error);
		
		static double saturate(double val, double min, double max);
		static double wrap(double val, double min, double max);
	
	private:
		double integral;
		double prev_error;
		double Kp;
		double Ki;
		double Kd;
		double ilimit;
};

#endif
