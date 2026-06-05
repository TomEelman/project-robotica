#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "pico/stdlib.h"

class PIDController {
public:
    PIDController(float kp, float ki, float kd,
                  float maxIntegral = 100.0f,
                  float maxOutput   = 255.0f);

    // Returns a control output in the range [-maxOutput, +maxOutput].
    float Compute(float currentValue, float setpoint);

    // Clears integrator, derivative state and resets the dt timer.
    void Reset();

private:
    float    kp, ki, kd;
    float    prevError;
    float    integral;
    float    maxIntegral;
    float    minOutput;
    float    maxOutput;
    uint64_t lastTime;
};

#endif