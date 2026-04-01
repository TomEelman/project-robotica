#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {

private:
	float kp;
	float ki;
	float kd;
	float setpoint;
	float prevError;
	float integral;
	int minPWM;
	int maxPWM;

public:
	PIDController(float p, float i, float d);

	float Compute(float CurrentValue, float Setpoint);

	void Reset();
};

#endif
