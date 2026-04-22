#ifndef DRIVE_H
#define DRIVE_H

#include "Motor.h"
#include "PIDController.h"
#include "SensorHub.h"
#include "DriveCommand.h"
#include "Kalmanfilter.h"

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

    bool  isTurning     = false;
    float turnStartYaw;
    float targetTurnYaw ;

    bool onTargetPos;

    float pwmLeft;
    float pwmRight; 

    float initialYaw;
    float encoderYaw;
    float targetYaw;
    bool isInitialYawSet;
    float currentAngular;
    float rampedLinear;
    float rampStep;

    void  ApplyClamp(float& pwm, float minPwm);
    float ComputeSteerCorrection();
    void  UpdateDirection(float linear, float angular);
    void  UpdateRamp(float linear);

public:
    Drive(Motor& LeftMotor, Motor& RightMotor,
          SensorHub& Sensors,
          float Wheelbase, int Threshold);

    bool TurnDegrees(float degrees);
    bool IsTurning() const { return isTurning; }

    void Execute(const DriveCommand& Command);

    void Stop();
};

#endif
