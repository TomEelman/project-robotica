#ifndef DRIVE_H
#define DRIVE_H

#include "Motor.h"
#include "PIDController.h"
#include "SensorHub.h"
#include "DriveCommand.h"

enum class DriveMode {
    Stopped,
    Forward,
    Backward,
    TurnLeft,
    TurnRight
};

class Drive {
public:
    Drive(Motor&     leftMotor,
          Motor&     rightMotor,
          SensorHub& sensors,
          float      wheelbaseMeters,
          float      minAngVel   = 13.0f,
          float      maxAngVel   = 35.0f,
          float      minPwmLeft  = 30.0f,
          float      minPwmRight = 30.0f);

    void Execute(const DriveCommand& command);
    void Stop();

private:
    Motor&     motorLeft;
    Motor&     motorRight;
    SensorHub& sensorHub;

    PIDController pidLeft;
    PIDController pidRight;
    PIDController pidYaw;

    float wheelbaseMeters;

    bool  enableMotorLeft;
    bool  enableMotorRight;

    DriveMode driveMode;

    float speedLeftFiltered;
    float speedRightFiltered;
    float lastOutputLeft;
    float lastOutputRight;
    bool  filterInitialized;

    float pwmLeft;
    float pwmRight;

    float targetYaw;
    bool  yawInitialized;

    float rampedLinear;
    float rampStep;

    float minAngVel;
    float maxAngVel;
    float minPwmLeft;
    float minPwmRight;
    int rampTick = 0;
    
    // Clamps |pwm| to [minPwm, 255]. Values below minPwm are zeroed to avoid
    // stalling the motor driver in a region where no movement occurs.
    void  ClampPwm(float& pwm, float minPwm);

    // Maps a percentage [-100, 100] to a PWM value [-255, 255].
    float PercentToPwm(float percent);

    // Smoothly accelerates rampedLinear toward the requested linear setpoint.
    // Prevents large inrush currents and wheel slip on hard starts.
    void UpdateRamp(float linearTarget);

    void UpdateDriveMode(float linear, float angular);

    // Computes a yaw-correction delta for straight-line driving.
    // Latches the target yaw on first call after a direction change and
    // uses the yaw PID to null out drift.
    float ComputeYawCorrection();

    void ExecuteTurn(float angular);
    void ExecuteLinear(float linear, float angular);
};

#endif