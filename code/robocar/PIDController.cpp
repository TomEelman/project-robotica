#include "PIDController.h"

PIDController::PIDController(float p, float i, float d) {
    kp       = p;
    ki       = i;
    kd       = d;
    prevError = 0.0f;
    integral  = 0.0f;
    setpoint  = 0.0f;
    minPWM   = -50;
    maxPWM   =  50;
}

float PIDController::Compute(float CurrentValue, float Setpoint) {
    uint64_t now = time_us_64();
    float dt = (now - lastTime) / 1000000.0f;

    // voorkom divide by zero
    if (dt <= 0.000001f) {
        dt = 0.000001f;
    }

    lastTime = now;

    float error = Setpoint - CurrentValue;

    // P
    float p = kp * error;

    // I
    integral += error * dt;
    float i = ki * integral;

    // D
    float derivative = (error - prevError) / dt;
    float d = kd * derivative;

    prevError = error;

    float output = p + i + d;

    // clamp (beter kleiner houden!)
    float maxOutput = 255.0f;
    if (output > maxOutput) output = maxOutput;
    if (output < -maxOutput) output = -maxOutput;

    return output;
}

void PIDController::Reset() {
    prevError = 0.0f;
    integral  = 0.0f;
    setpoint  = 0.0f;
}
