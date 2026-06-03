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
          float      maxAngVel   = 40.0f,
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
    float rampedTurnSpeed;

    // Smoothed version of the incoming linear command.
    // Limits how fast the navigator can change the requested speed per tick,
    // preventing abrupt jumps like 278 → 0 from reaching the ramp in one step.
    float cmdSmoothedLinear;

    float minAngVel;
    float maxAngVel;
    float minPwmLeft;
    float minPwmRight;

    // ── Reversal lockout ────────────────────────────────────────────────────
    // Single-channel encoders measure magnitude only — never direction.
    // When the drive mode flips from Forward→Backward or Backward→Forward,
    // the robot still rolls in the old direction due to inertia while the
    // command sign has already changed. EncoderMetTeken on the Pi5 would then
    // assign the wrong sign to the measured speed, causing a position jump in
    // the localisation that smears the map.
    //
    // Fix: on a reversal, hold the motors at zero (non-blocking) until both
    // encoders read below REVERSAL_STILL_MM_S, then continue with the new
    // direction. A safety timeout (REVERSAL_TIMEOUT_MS) prevents stalling
    // indefinitely if the encoders are noisy.
    bool     reversalLockout;
    uint32_t reversalStartMs;

    // Clamps |pwm| to [minPwm, 255]. Values below minPwm are zeroed to avoid
    // stalling the motor driver in a region where no movement occurs.
    void  ClampPwm(float& pwm, float minPwm);

    // Maps a percentage [-100, 100] to a PWM value [-255, 255].
    float PercentToPwm(float percent);

    // Smoothly accelerates rampedLinear toward the requested linear setpoint.
    void UpdateRamp(float linearTarget);

    void UpdateDriveMode(float linear, float angular);

    // Computes a yaw-correction delta for straight-line driving.
    float ComputeYawCorrection();

    void ExecuteTurn(float angular);
    void ExecuteLinear(float linear, float angular);
};

#endif