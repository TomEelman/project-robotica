// Drive.cpp
#include "Drive.h"
#include <cstdio>

Drive::Drive(Motor& LeftMotor, Motor& RightMotor, SensorHub& Sensors, float Wheelbase, int Threshold)
    : motorLeft(LeftMotor),
      motorRight(RightMotor),
      sensorHub(Sensors),
      pIDLeft(0.141f, 1.763f, 0.0074f),
      pIDRight(0.135f, 1.688f, 0.0071f),
      pIDYaw(7.0f, 0.0f, 0.f),
      wheelbase(Wheelbase),
      threshold(Threshold),
      enableMotorA(true),
      enableMotorB(true),
      motorDirection(STOPPED),
      onTargetPos(false),
      pwmLeft(50.0f),
      pwmRight(50.0f)
{
}


void Drive::Execute(const DriveCommand& Command) {
    float linear  = Command.GetLinVelocity();
    float angular = Command.GetAngVelocity();

    // 1. Stop logica & Reset
    if (linear == 0 && angular == 0) {
        isInitialYawSet = false;
        rampedLinear = 0.0f; // Reset ook de ramp bij stop
        Stop();
        return;
    }

    // 2. Ramping Logica: Verhoog de snelheid geleidelijk
    // Pas de 2.0f aan voor een snellere of langzamere ramp
    const float rampStep = 2.0f; 
    if (rampedLinear < linear) {
        rampedLinear += rampStep;
        if (rampedLinear > linear) rampedLinear = linear;
    } else if (rampedLinear > linear) {
        rampedLinear -= rampStep; // Ook netjes afremmen
        if (rampedLinear < linear) rampedLinear = linear;
    }

    // 3. Yaw Target vastleggen
    if (!isInitialYawSet) {
        initialYaw = sensorHub.GetCurrentYaw();
        isInitialYawSet = true;
        pIDYaw.Reset();
    }

    float currentYaw = sensorHub.GetCurrentYaw();
    float targetYaw  = initialYaw;

    // 4. Shortest path berekening
    float yawError = targetYaw - currentYaw;
    while (yawError > 180.0f)  yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    // 5. Yaw PID berekening
    float steerCorrection = pIDYaw.Compute(0, -yawError); 

    // 6. Targets berekenen (Gebruik rampedLinear in plaats van linear!)
    float targetLeft  = rampedLinear - (angular * wheelbase / 2.0f) - steerCorrection;
    float targetRight = rampedLinear + (angular * wheelbase / 2.0f) + steerCorrection;

    float currentLeft  = sensorHub.GetSpeedLeft();
    float currentRight = sensorHub.GetSpeedRight();

    // Richting bepalen op basis van de opgebouwde snelheid
    if (rampedLinear > threshold) {
        motorDirection = FORWARD;
    } else if (rampedLinear < -threshold) {
        motorDirection = BACKWARD;
    } else if (angular != 0) {
        motorDirection = (angular > 0) ? TURN_RIGHT : TURN_LEFT;
    }

    // 7. Wiel PID correctie (Inner loop)
    float correctionLeft  = pIDLeft.Compute(currentLeft,  targetLeft);
    float correctionRight = pIDRight.Compute(currentRight, targetRight);

    pwmLeft  = correctionLeft;
    pwmRight = correctionRight;

    // Clampen
    if (pwmLeft  > 255.0f) pwmLeft  = 255.0f;
    if (pwmLeft  <  15.0f) pwmLeft  =  15.0f;
    if (pwmRight > 255.0f) pwmRight = 255.0f;
    if (pwmRight <  15.0f) pwmRight =  15.0f;

    // Debugging (RampedLinear toegevoegd)
    printf("RampSpeed: %.2f | Yaw: %.2f | Steer: %.2f\n", rampedLinear, currentYaw, steerCorrection);
    printf("Speed L: %.2f | Target L: %.2f | PWM L: %.2f\n", currentLeft, targetLeft, pwmLeft);
    printf("Speed R: %.2f | Target R: %.2f | PWM R: %.2f\n", currentRight, targetRight, pwmRight);

    if (enableMotorA) motorLeft.SetSpeed(pwmLeft);
    if (enableMotorB) motorRight.SetSpeed(pwmRight);
}


void Drive::Stop() {
    motorDirection = STOPPED;
    onTargetPos    = true;
    pwmLeft        = 50.0f;  // reset naar startwaarde
    pwmRight       = 50.0f;

    motorLeft.Stop();
    motorRight.Stop();

    pIDLeft.Reset();
    pIDRight.Reset();
}