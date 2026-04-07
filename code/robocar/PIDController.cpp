#include "PIDController.h"

PIDController::PIDController(float p, float i, float d) {
    kp       = p;
    ki       = i;
    kd       = d;
    prevError = 0.0f;
    integral  = 0.0f;
    setpoint  = 0.0f;
    minPWM   = -100;
    maxPWM   =  100;
}

float PIDController::Compute(float CurrentValue, float Setpoint) {
    float error = Setpoint - CurrentValue;

    // P: proportioneel aan de huidige fout
    float p = kp * error;

    // I: opgebouwde fout over tijd
    integral += error;
    float i = ki * integral;

    // D: verandering van de fout (dempend)
    float derivative = error - prevError;
    float d = kd * derivative;

    prevError = error;

    // Output berekenen en clampen tussen minPWM en maxPWM
    float output = p + i + d;
    if (output > maxPWM) output = maxPWM;
    if (output < minPWM) output = minPWM;

    return output;
}

void PIDController::Reset() {
    prevError = 0.0f;
    integral  = 0.0f;
    setpoint  = 0.0f;
}
