#include "Drive.h"
#include <cstdio>

Drive::Drive(Motor& LeftMotor, Motor& RightMotor, SensorHub& Sensors, float Wheelbase, int Threshold)
    : motorLeft(LeftMotor),
      motorRight(RightMotor),
      sensorHub(Sensors),
      pIDLeft(0.2f, 0.1f, 0.00f),
      pIDRight(0.2f, 0.1f, 0.00f),
      wheelbase(Wheelbase),
      threshold(Threshold),
      enableMotorA(true),
      enableMotorB(true),
      motorDirection(STOPPED),
      onTargetPos(false)
{
}

void Drive::Execute(const DriveCommand& Command) {
    float linear  = Command.GetLinVelocity();
    float angular = Command.GetAngVelocity();

    // Differentieel rijden:
    // Gewenste snelheid per wiel berekenen op basis van
    // lineaire snelheid en hoeksnelheid (wheelbase in meter)
    float targetLeft  = linear - (angular * wheelbase / 2.0f);
    float targetRight = linear + (angular * wheelbase / 2.0f);

    // Huidige snelheid ophalen van encoders
    float currentLeft  = sensorHub.GetSpeedLeft();
    float currentRight = sensorHub.GetSpeedRight();

    float outputLeft  = pIDLeft.Compute(currentLeft, targetLeft);
    float outputRight = pIDRight.Compute(currentRight, targetRight);

 // elke 10 iteraties
    printf("Speed L: %.2f | Target L: %.2f\n", currentLeft, targetLeft);
    printf("Speed R: %.2f | Target R: %.2f\n", currentRight, targetRight);

    
    // Rijrichting bepalen op basis van gewenste snelheden
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

    // Motoren aansturen
    if (motorDirection == STOPPED) {
        Stop();
        return;
    }
    printf("PWM L: %f PWM R: %f\n", outputLeft, outputRight);
    if (enableMotorA) motorLeft.SetSpeed(outputLeft);
    if (enableMotorB) motorRight.SetSpeed(outputRight);
}

void Drive::Stop() {
    motorDirection = STOPPED;
    onTargetPos    = true;

    motorLeft.Stop();
    motorRight.Stop();

    pIDLeft.Reset();
    pIDRight.Reset();
}