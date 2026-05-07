#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "pico/stdlib.h"

class PIDController {
public:
    PIDController(float kp, float ki, float kd,
                  float maxIntegral = 100.0f,
                  float maxOutput   = 255.0f);

    // Returns a control output in the range [-maxOutput, +maxOutput].
    // Call once per control loop tick with the current measured value
    // and the desired setpoint.
    float Compute(float currentValue, float setpoint);

    // Clears integrator, derivative state and resets the dt timer.
    // Call whenever the controller is taken out of the loop (motor stop,
    // mode switch) to prevent integrator wind-up on re-entry.
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