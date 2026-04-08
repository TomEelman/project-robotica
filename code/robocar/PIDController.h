#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include "pico/stdlib.h"
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
	uint64_t lastTime;

public:
	PIDController(float p, float i, float d);

	float Compute(float CurrentValue, float Setpoint);

	void Reset();
};

#endif
