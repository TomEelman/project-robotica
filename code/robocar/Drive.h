#ifndef DRIVE_H
#define DRIVE_H

#include "Motor.h"
#include "PIDController.h"
#include "SensorHub.h"
#include "DriveCommand.h"

enum Drivemodes {
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    STOPPED
};

class Drive {

private:
    Motor& motorLeft;
    Motor& motorRight;

    PIDController pIDLeft;
    PIDController pIDRight;
    PIDController pIDYaw;
    SensorHub& sensorHub;

    float wheelbase;
    int threshold;

    bool enableMotorA;
    bool enableMotorB;

    Drivemodes motorDirection;
    bool onTargetPos;

    float pwmLeft;   // huidige PWM waarde links
    float pwmRight;  // huidige PWM waarde rechts

    float initialYaw;
    float encoderYaw;
    float targetYaw;
    bool isInitialYawSet;

    float rampedLinear;
    float rampStep;
public:
    Drive(Motor& LeftMotor, Motor& RightMotor,
          SensorHub& Sensors,
          float Wheelbase, int Threshold);

    void Execute(const DriveCommand& Command);

    void Stop();
};

#endif
