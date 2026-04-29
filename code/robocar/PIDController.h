// PIDController.h
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
    float minOutput;
    float maxOutput;
    float maxIntegral;
    uint64_t lastTime;
    float dt;

public:
    PIDController(float p, float i, float d, float maxIntegral = 100.0f, float maxOutput = 255.0f);
    float Compute(float CurrentValue, float Setpoint);
    void Reset();
};

#endif