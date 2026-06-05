#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd,
                             float maxIntegral, float maxOutput)
    : kp(kp),
      ki(ki),
      kd(kd),
      prevError(0.0f),
      integral(0.0f),
      maxIntegral(maxIntegral),
      minOutput(-maxOutput),
      maxOutput(maxOutput),
      lastTime(time_us_64())
{
}

float PIDController::Compute(float currentValue, float setpoint)
{
    uint64_t now = time_us_64();
    float    dt  = static_cast<float>(now - lastTime) / 1'000'000.0f;
    lastTime = now;

    float error = setpoint - currentValue;

    float p = kp * error;

    integral += error * dt;
    if (integral >  maxIntegral) integral =  maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;
    float i = ki * integral;

    // Derivative on error (not measurement) because setpoints change smoothly via the ramp in drive.
    float derivative = (dt > 0.0f) ? (error - prevError) / dt : 0.0f;
    float d = kd * derivative;

    prevError = error;

    float output = p + i + d;
    if (output >  maxOutput) output =  maxOutput;
    if (output < -maxOutput) output = -maxOutput;

    return output;
}

void PIDController::Reset()
{
    prevError = 0.0f;
    integral  = 0.0f;
    lastTime  = time_us_64();
}