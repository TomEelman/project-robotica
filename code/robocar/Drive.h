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
    Motor&    motorLeft;
    Motor&    motorRight;
    SensorHub& sensorHub;

    PIDController pIDLeft;
    PIDController pIDRight;
    PIDController pIDYaw;
    

    float wheelbase;
    int   threshold;

    bool enableMotorA;
    bool enableMotorB;

    Drivemodes motorDirection;

    float speedLFilt = 0.0f;
    float speedRFilt = 0.0f;
    float lastOutLeft = 0.0f;
    float lastOutRight = 0.0f;
    bool  filtInit = false;

    float pwmLeft;
    float pwmRight;

    float initialYaw;
    float targetYaw;
    bool  isInitialYawSet;
    float currentAngular;
    float rampedLinear;
    float rampStep;

    float minAngVel;
    float maxAngVel;
 
    float minPwmLeft;
    float minPwmRight;

    void  ApplyClamp(float& pwm, float minPwm);
    float PercentToPwm(float percent, float minPwm);
    float ComputeSteerCorrection();
    float ComputeEncoderAngVel() const;
    void  UpdateDirection(float linear, float angular);
    void  UpdateRamp(float linear);

public:
    Drive(Motor& LeftMotor, Motor& RightMotor,
          SensorHub& Sensors,
          float Wheelbase, int Threshold,
          float MinAngVel  = 13.0f,
          float MaxAngVel  = 35.0f,
          float MinPwmLeft = 80.0f,
          float MinPwmRight= 80.0f);

    void Execute(const DriveCommand& Command);
    void Stop();
};

#endif