// Drive.cpp
#include "Drive.h"
#include <cstdio>

Drive::Drive(Motor& LeftMotor, Motor& RightMotor, SensorHub& Sensors, float Wheelbase, int Threshold)
    : motorLeft(LeftMotor),
      motorRight(RightMotor),
      sensorHub(Sensors),
      pIDLeft(0.4f, 0.5f, 0.0f),
      pIDRight(0.4f, 0.5f, 0.0f),
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

    float targetLeft  = linear - (angular * wheelbase / 2.0f);
    float targetRight = linear + (angular * wheelbase / 2.0f);

    float currentLeft  = sensorHub.GetSpeedLeft();
    float currentRight = sensorHub.GetSpeedRight();

    // richting bepalen
    if (linear > threshold) {
        motorDirection = FORWARD;
    } else if (linear < -threshold) {
        motorDirection = BACKWARD;
    } else if (angular > 0) {
        motorDirection = TURN_RIGHT;
    } else if (angular < 0) {
        motorDirection = TURN_LEFT;
    } else {
        motorDirection = STOPPED;
    }

    if (motorDirection == STOPPED) {
        Stop();
        return;
    }

    // PID correctie berekenen
    float correctionLeft  = pIDLeft.Compute(currentLeft,  targetLeft);
    float correctionRight = pIDRight.Compute(currentRight, targetRight);

    // PWM bijsturen
    pwmLeft  = correctionLeft;
    pwmRight = correctionRight;

    // clamp tussen drempelwaarde en max
    if (pwmLeft  > 255.0f) pwmLeft  = 255.0f;
    if (pwmLeft  <  15.0f) pwmLeft  =  15.0f;
    if (pwmRight > 255.0f) pwmRight = 255.0f;
    if (pwmRight <  15.0f) pwmRight =  15.0f;

    printf("Speed L: %.2f | Target L: %.2f\n", currentLeft,  targetLeft);
    printf("Speed R: %.2f | Target R: %.2f\n", currentRight, targetRight);
    printf("PWM L:   %.2f | PWM R:   %.2f\n",  pwmLeft,      pwmRight);

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