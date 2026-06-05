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
          float      minAngVel   = 13.0f, // numbers to prevent stalling and slipping
          float      maxAngVel   = 50.0f,
          float      minPwmLeft  = 35.0f, 
          float      minPwmRight = 35.0f);

    void Execute(const DriveCommand& command); // gets the drivecommand and directs it to the right function
    void Stop(); // stops the motors and sets the data right

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
    float rampedTurnSpeed;

    // Smoothed version of the incoming linear command.
    // preventing abrupt jumps like 278 to 0
    float cmdSmoothedLinear;

    float minAngVel;
    float maxAngVel;
    float minPwmLeft;
    float minPwmRight;

    // prevents smearing of the map on the pi5 due to the sign
    bool     reversalLockout;
    uint32_t reversalStartMs;

    // Clamps pwm to [minPwm, 255]
    void  ClampPwm(float& pwm, float minPwm);

    // Maps a percentage [-100, 100] to a PWM value [-255, 255].
    float PercentToPwm(float percent);

    // Smoothly accelerates rampedLinear toward the requested linear setpoint.
    void UpdateRamp(float linearTarget);

    // for the drivemodes 
    void UpdateDriveMode(float linear, float angular); 

    // Computes a yaw-correction delta for straight-line driving.
    float ComputeYawCorrection();

    //turn on position
    void ExecuteTurn(float angular);

    // linear holds his angle more or less
    void ExecuteLinear(float linear, float angular);
};

#endif