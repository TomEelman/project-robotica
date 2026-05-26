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
    float lastLimitedPwmLeft;
    float lastLimitedPwmRight;

    float targetYaw;
    bool  yawInitialized;

    float rampedLinear;
    float rampStep;

    float minAngVel;
    float maxAngVel;
    float minPwmLeft;
    float minPwmRight;
    int   rampTick = 0;

    // Clamps pwm, values below minPwm are set to zero to avoid stalling (no motor movement)
    
    void clampPwm(float& pwm, float minPwm);

    // Maps a percentage +-100 to +-255
    float percentToPwm(float percent);


    void updateRamp(float linearTarget);
    float applyOutputLimiter(float target, float previous);
    void updateDriveMode(float linear, float angular);

    // Computes a yaw-correction delta for straight-line driving.
    // saves the target yaw on first call after a direction change and
    // uses the yaw PID to keep straight
    float computeYawCorrection();

    void executeTurn(float angular);
    void executeLinear(float linear, float angular);

};

#endif