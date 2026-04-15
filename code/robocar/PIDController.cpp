// PIDController.cpp
#include "PIDController.h"

PIDController::PIDController(float p, float i, float d, float maxIntegral_, float maxOutput_) {
    kp           = p;
    ki           = i;
    kd           = d;
    prevError    = 0.0f;
    integral     = 0.0f;
    setpoint     = 0.0f;
    maxIntegral  = maxIntegral_;
    minOutput    = -maxOutput_;
    maxOutput    =  maxOutput_;
    lastTime     = time_us_64();

}

float PIDController::Compute(float CurrentValue, float Setpoint, float dt) {


    float error = Setpoint - CurrentValue;

    // P
    float p = kp * error;

    // I met anti-windup
    integral += error * dt;
    if (integral >  maxIntegral) integral =  maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;
    float i = ki * integral;

    // D
    float derivative = (error - prevError) / dt;
    float d = kd * derivative;

    prevError = error;

    float output = p + i + d;

    if (output >  maxOutput) output =  maxOutput;
    if (output < -maxOutput) output = -maxOutput;

    return output;
}

void PIDController::Reset() {
    prevError = 0.0f;
    integral  = 0.0f;
    setpoint  = 0.0f;
    lastTime  = time_us_64();
}