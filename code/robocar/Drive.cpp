#include "Drive.h"
#include <cstdio>
#include <cmath>

// ----------------------------------------
// CONSTRUCTOR
// ----------------------------------------

Drive::Drive(Motor& LeftMotor, Motor& RightMotor,
             SensorHub& Sensors,
             float Wheelbase, int Threshold,
             float MinAngVel,
             float MaxAngVel,
             float MinPwmLeft,
             float MinPwmRight)
    : motorLeft(LeftMotor),
      motorRight(RightMotor),
      sensorHub(Sensors),
      pIDLeft(0.145f, 1.763f, 0.0074f, 150.f, 255.0f),
      pIDRight(0.135f, 1.688f, 0.0074f, 150.f, 255.0f),
      pIDYaw(4.0f, 0.1f, 0.3f, 400.f, 255.0f),
      wheelbase(Wheelbase),
      threshold(Threshold),
      enableMotorA(true),
      enableMotorB(true),
      motorDirection(STOPPED),
      isTurning(false),
      turnStartYaw(0.0f),
      targetTurnYaw(0.0f),
      onTargetPos(false),
      pwmLeft(0.0f),
      pwmRight(0.0f),
      initialYaw(0.0f),
      encoderYaw(0.0f),
      targetYaw(0.0f),
      isInitialYawSet(false),
      currentAngular(0.0f),
      rampedLinear(0.0f),
      rampStep(2.0f),
      minAngVel(MinAngVel),
      maxAngVel(MaxAngVel),
      minPwmLeft(MinPwmLeft),
      minPwmRight(MinPwmRight)
{
}

// ----------------------------------------
// PRIVATE HELPERS
// ----------------------------------------

void Drive::ApplyClamp(float& pwm, float minPwm)
{
    if (fabs(pwm) < 15.0f)
    {
        pwm = 0.0f;
    }
    else
    {
        if (pwm >  255.0f) pwm =  255.0f;
        if (pwm < -255.0f) pwm = -255.0f;
        if (pwm > 0.0f && pwm <  minPwm) pwm =  minPwm;
        if (pwm < 0.0f && pwm > -minPwm) pwm = -minPwm;
    }
}

float Drive::ComputeEncoderAngVel() const
{
    float speedL = sensorHub.GetSpeedLeft();
    float speedR = sensorHub.GetSpeedRight();
    // wheelbase is in meters, speeds in mm/s → convert wheelbase to mm
    float wheelbaseMm = wheelbase * 1000.0f;
    return (speedR - speedL) / wheelbaseMm * (180.0f / M_PI);
}

void Drive::UpdateDirection(float linear, float angular)
{
    if (fabs(linear) < 0.001f && fabs(angular) > 0.001f)
        motorDirection = (angular > 0.0f) ? TURN_RIGHT : TURN_LEFT;
    else if (linear >  0.001f)
        motorDirection = FORWARD;
    else if (linear < -0.001f)
        motorDirection = BACKWARD;
    else
        motorDirection = STOPPED;
}

void Drive::UpdateRamp(float linear)
{
    switch (motorDirection)
    {
        case FORWARD:
            if (rampedLinear < minPwmLeft)
                rampedLinear = minPwmLeft;
            rampedLinear += rampStep;
            if (rampedLinear > linear) rampedLinear = linear;
            break;

        case BACKWARD:
            if (rampedLinear > -minPwmLeft)
                rampedLinear = -minPwmLeft;
            rampedLinear -= rampStep;
            if (rampedLinear < linear) rampedLinear = linear;
            break;

        case TURN_LEFT:
        case TURN_RIGHT:
        case STOPPED:
            rampedLinear = 0.0f;
            break;
    }
}

float Drive::ComputeSteerCorrection()
{
    if (!isInitialYawSet)
    {
        initialYaw      = sensorHub.GetCurrentYaw();
        targetYaw       = initialYaw;
        isInitialYawSet = true;
        pIDYaw.Reset();
    }

    targetYaw += currentAngular * 0.01f;

    float currentYaw = sensorHub.GetCurrentYaw();
    float yawError   = targetYaw - currentYaw;
    while (yawError >  180.0f) yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    return pIDYaw.Compute(0.0f, -yawError);
}

// ----------------------------------------
// EXECUTE
// ----------------------------------------

void Drive::Execute(const DriveCommand& Command)
{
    float linear  = Command.GetLinVelocity();
    float angular = Command.GetAngVelocity();
    currentAngular = angular;

    // STOP
    if (fabs(linear) < 0.001f && fabs(angular) < 0.001f)
    {
        isInitialYawSet = false;
        rampedLinear    = 0.0f;
        pIDLeft.Reset();
        pIDRight.Reset();
        pIDYaw.Reset();
        Stop();
        return;
    }

    UpdateDirection(linear, angular);
    UpdateRamp(linear);

    float steerCorrection = ComputeSteerCorrection();

    float pwmL = 0.0f;
    float pwmR = 0.0f;

    switch (motorDirection)
    {
        case TURN_RIGHT:
        {
            float targetAngVel = fabs(angular);
            if (targetAngVel < minAngVel) targetAngVel = minAngVel;
            if (targetAngVel > maxAngVel) targetAngVel = maxAngVel;

            float currentAngVelAbs = fabs(ComputeEncoderAngVel());

            float basePwm = minPwmLeft + (targetAngVel - minAngVel)
                            / (maxAngVel - minAngVel)
                            * (255.0f - minPwmLeft);
            float error   = targetAngVel - currentAngVelAbs;
            float pwmTurn = basePwm + 0.5f * error;

            if (pwmTurn > 255.0f)    pwmTurn = 255.0f;
            if (pwmTurn < minPwmLeft) pwmTurn = minPwmLeft;

            pwmL =  pwmTurn;
            pwmR = -pwmTurn;

            ApplyClamp(pwmL, minPwmLeft);
            ApplyClamp(pwmR, minPwmRight);
            break;
        }

        case TURN_LEFT:
        {
            float targetAngVel = fabs(angular);
            if (targetAngVel < minAngVel) targetAngVel = minAngVel;
            if (targetAngVel > maxAngVel) targetAngVel = maxAngVel;

            float currentAngVelAbs = fabs(ComputeEncoderAngVel());

            float basePwm = minPwmLeft + (targetAngVel - minAngVel)
                            / (maxAngVel - minAngVel)
                            * (255.0f - minPwmLeft);
            float error   = targetAngVel - currentAngVelAbs;
            float pwmTurn = basePwm + 0.5f * error;

            if (pwmTurn > 255.0f)    pwmTurn = 255.0f;
            if (pwmTurn < minPwmLeft) pwmTurn = minPwmLeft;

            pwmL = -pwmTurn;
            pwmR =  pwmTurn;

            ApplyClamp(pwmL, minPwmLeft);
            ApplyClamp(pwmR, minPwmRight);
            break;
        }

        case FORWARD:
        {
            float targetLeft  = rampedLinear - steerCorrection;
            float targetRight = rampedLinear + steerCorrection;

            pwmL = pIDLeft.Compute(sensorHub.GetSpeedLeft(),  targetLeft);
            pwmR = pIDRight.Compute(sensorHub.GetSpeedRight(), targetRight);

            ApplyClamp(pwmL, minPwmLeft);
            ApplyClamp(pwmR, minPwmRight);
            break;
        }

        case BACKWARD:
        {
            float targetLeft  = rampedLinear - steerCorrection;
            float targetRight = rampedLinear + steerCorrection;

            pwmL = pIDLeft.Compute(-sensorHub.GetSpeedLeft(),  targetLeft);
            pwmR = pIDRight.Compute(-sensorHub.GetSpeedRight(), targetRight);

            ApplyClamp(pwmL, minPwmLeft);
            ApplyClamp(pwmR, minPwmRight);
            break;
        }

        default:
            break;
    }

    pwmLeft  = pwmL;
    pwmRight = pwmR;

    if (enableMotorA) motorLeft.SetSpeed(pwmLeft);
    if (enableMotorB) motorRight.SetSpeed(pwmRight);
}

// ----------------------------------------
// STOP
// ----------------------------------------

void Drive::Stop()
{
    motorDirection = STOPPED;
    onTargetPos    = true;
    pwmLeft        = 0.0f;
    pwmRight       = 0.0f;
    motorLeft.Stop();
    motorRight.Stop();
    pIDLeft.Reset();
    pIDRight.Reset();
    pIDYaw.Reset();
}