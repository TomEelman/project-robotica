#include "Drive.h"
#include <cstdio>
#include <cmath>
Drive::Drive(Motor& LeftMotor, Motor& RightMotor, SensorHub& Sensors, float Wheelbase, int Threshold)
    : motorLeft(LeftMotor),
      motorRight(RightMotor),
      sensorHub(Sensors),
      pIDLeft(0.145f, 1.763f, 0.0074f, 150.f, 255.0f),
      pIDRight(0.135f, 1.688f, 0.0074f,150.f, 255.0f),
      pIDYaw(4.0f, 0.1f, 0.3f,400.f, 255.0f),
      wheelbase(Wheelbase),
      threshold(Threshold),
      enableMotorA(true),
      enableMotorB(true),
      motorDirection(STOPPED),
      onTargetPos(false),
      pwmLeft(50.0f),
      pwmRight(50.0f)
{
}



void Drive::Execute(const DriveCommand& Command, float dt) {
    float linear  = Command.GetLinVelocity();
    float angular = Command.GetAngVelocity();
    static float Kv = 255.0f / linear;

    if (linear == 0 && angular == 0) {
        isInitialYawSet = false;
        rampedLinear = 0.0f;
        pIDYaw.Reset(); 
        Stop();
        return;
    }

    
    const float rampStep = 2.0f; 
    if (rampedLinear < linear) {
        rampedLinear += rampStep;
        if (rampedLinear > linear) rampedLinear = linear;
    } else if (rampedLinear > linear) {
        rampedLinear -= rampStep;
        if (rampedLinear < linear) rampedLinear = linear;
    }


    if (!isInitialYawSet) {
        initialYaw = sensorHub.GetCurrentYaw();
        targetYaw  = initialYaw;
        isInitialYawSet = true;
        pIDYaw.Reset();
    }


    targetYaw += angular * 0.01f; 

    float currentYaw = sensorHub.GetCurrentYaw();
    float yawError = targetYaw - currentYaw;

    while (yawError > 180.0f)  yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    float steerCorrection = pIDYaw.Compute(0, -yawError, dt); 


    float targetLeft  = (rampedLinear - steerCorrection);
    float targetRight = rampedLinear + steerCorrection;


    float currentLeft  = sensorHub.GetSpeedLeft();
    float currentRight = sensorHub.GetSpeedRight();

    float correctionLeft  = pIDLeft.Compute(currentLeft,  targetLeft, dt);
    float correctionRight = pIDRight.Compute(currentRight, targetRight, dt);


    pwmLeft  = correctionLeft;
    pwmRight = correctionRight;

    auto applyClamp = [](float& pwm, float minPwm) {
        if (abs(pwm) < 15.0f) { 
            pwm = 0;
        } else {
            if (pwm > 255.0f) pwm = 255.0f;
            if (pwm < -255.0f) pwm = -255.0f; 
            

            if (pwm > 0 && pwm < minPwm) pwm = minPwm;
            if (pwm < 0 && pwm > -minPwm) pwm = -minPwm;
        }
    };

    applyClamp(pwmLeft, 23.0f);
    applyClamp(pwmRight, 22.5f);


    if (enableMotorA) motorLeft.SetSpeed(pwmLeft);
    if (enableMotorB) motorRight.SetSpeed(pwmRight);
}

void Drive::Stop() {
    motorDirection = STOPPED;
    onTargetPos    = true;
    pwmLeft        = 50.0f;
    pwmRight       = 50.0f;

    motorLeft.Stop();
    motorRight.Stop();

    pIDLeft.Reset();
    pIDRight.Reset();
}

