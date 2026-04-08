// PIDController.cpp
#include "PIDController.h"

PIDController::PIDController(float p, float i, float d) {
    kp        = p;
    ki        = i;
    kd        = d;
    prevError = 0.0f;
    integral  = 0.0f;
    setpoint  = 0.0f;
    minPWM = -255.0f;  // maximale correctie per stap
    maxPWM =  255.0f;
    lastTime  = time_us_64();
}

float PIDController::Compute(float CurrentValue, float Setpoint) {
    uint64_t now = time_us_64();
    float dt = (now - lastTime) / 1000000.0f;

    if (dt <= 0.000001f) dt = 0.000001f;

    lastTime = now;

    float error = Setpoint - CurrentValue;

    // P
    float p = kp * error;

    // I met anti-windup
    integral += error * dt;
    if (integral >  100.0f) integral =  100.0f;
    if (integral < -100.0f) integral = -100.0f;
    float i = ki * integral;

    // D
    float derivative = (error - prevError) / dt;
    float d = kd * derivative;

    prevError = error;

    float output = p + i + d;

    if (output >  maxPWM) output =  maxPWM;
    if (output < -maxPWM) output = -maxPWM;

    return output;
}

void PIDController::Reset() {
    prevError = 0.0f;
    integral  = 0.0f;
    setpoint  = 0.0f;
    lastTime  = time_us_64();
}