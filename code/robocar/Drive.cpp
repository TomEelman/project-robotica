#include "Drive.h"
#include <cstdio>
#include <cmath>

Drive::Drive(Motor& LeftMotor, Motor& RightMotor, SensorHub& Sensors, float Wheelbase, int Threshold)
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
      onTargetPos(false),
      pwmLeft(0.0f),
      pwmRight(0.0f),
      rampedLinear(0.0f)
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
    const float rampStep = 2.0f;
    const float minStart = 23.0f;

    switch (motorDirection)
    {
        case FORWARD:
            if (rampedLinear < minStart)
                rampedLinear = minStart;
            rampedLinear += rampStep;
            if (rampedLinear > linear) rampedLinear = linear;
            break;

        case BACKWARD:
            if (rampedLinear > -minStart)
                rampedLinear = -minStart;
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
    currentAngular = angular;  // bewaar voor ComputeSteerCorrection

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
        float targetAngVel  =  fabs(angular);
        float currentAngVel = sensorHub.GetAngVelocity();
        printf("target%f\n,current%f\n",targetAngVel, currentAngVel);
        float pwmTurn = pIDYaw.Compute(currentAngVel, targetAngVel);

        float mag = fabs(pwmTurn);
        if (mag > 255.0f) mag = 255.0f;
        if (mag <  23.0f) mag =  23.0f;

        pwmL =  mag;
        pwmR = -mag;
        printf("PWML%f\n,PWMR%f\n",pwmL, pwmR);
        ApplyClamp(pwmL, 23.0f);
        ApplyClamp(pwmR, 22.5f);
        break;
    }

    case TURN_LEFT:
    {
        float targetAngVel  = -fabs(angular);
        float currentAngVel = sensorHub.GetAngVelocity();
        printf("target%f\n,current%f\n",targetAngVel, currentAngVel);
        float pwmTurn = pIDYaw.Compute(currentAngVel, targetAngVel);

        float mag = fabs(pwmTurn);
        if (mag > 255.0f) mag = 255.0f;
        if (mag <  23.0f) mag =  23.0f;

        pwmL = -mag;
        pwmR =  mag;
        printf("PWML%f\n,PWMR%f\n",pwmL, pwmR);
        ApplyClamp(pwmL, 23.0f);
        ApplyClamp(pwmR, 22.5f);
        break;
    }

    
        case FORWARD:
        {
            float targetLeft  = rampedLinear - steerCorrection;
            float targetRight = rampedLinear + steerCorrection;

            pwmL = pIDLeft.Compute(sensorHub.GetSpeedLeft(),  targetLeft);
            pwmR = pIDRight.Compute(sensorHub.GetSpeedRight(), targetRight);

            ApplyClamp(pwmL, 23.0f);
            ApplyClamp(pwmR, 22.5f);
            break;
        }

        case BACKWARD:
        {
            float targetLeft  = rampedLinear - steerCorrection;
            float targetRight = rampedLinear + steerCorrection;

            // Encoder unsigned → negatief maken voor correcte PID feedback
            pwmL = pIDLeft.Compute(-sensorHub.GetSpeedLeft(),  targetLeft);
            pwmR = pIDRight.Compute(-sensorHub.GetSpeedRight(), targetRight);

            ApplyClamp(pwmL, 23.0f);
            ApplyClamp(pwmR, 22.5f);
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