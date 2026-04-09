

// Drive.cpp
#include "Drive.h"
#include <cstdio>
#include <cmath>
Drive::Drive(Motor& LeftMotor, Motor& RightMotor, SensorHub& Sensors, float Wheelbase, int Threshold)
    : motorLeft(LeftMotor),
      motorRight(RightMotor),
      sensorHub(Sensors),
      pIDLeft(0.135f, 1.0f, 0.0071f),
      pIDRight(0.135f, 1.0f, 0.0071f),
      pIDYaw(3.8f, 1.0f, 1.4f),
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
void Drive::Execute(const DriveCommand& Command) {
    float linear  = Command.GetLinVelocity();
    float angular = Command.GetAngVelocity();

    // 1. Stop logica: alles resetten als er geen commando is
    if (linear == 0 && angular == 0) {
        isInitialYawSet = false;
        rampedLinear = 0.0f;
        pIDYaw.Reset(); // Reset ook de integrator van de PID
        Stop();
        return;
    }

    // 2. Up- en downrampen van snelheid voor vloeiende beweging
    const float rampStep = 2.0f; 
    if (rampedLinear < linear) {
        rampedLinear += rampStep;
        if (rampedLinear > linear) rampedLinear = linear;
    } else if (rampedLinear > linear) {
        rampedLinear -= rampStep;
        if (rampedLinear < linear) rampedLinear = linear;
    }

    // 3. Initialiseren van de koers (Target)
    if (!isInitialYawSet) {
        initialYaw = sensorHub.GetCurrentYaw();
        targetYaw  = initialYaw;
        isInitialYawSet = true;
        pIDYaw.Reset();
    }

    // 4. Target bijwerken op basis van draaicommando
    // 0.01f moet overeenkomen met je loop-tijd (bijv. 10ms)
    targetYaw += angular * 0.01f; 

    // 5. Yaw error berekenen met Shortest Path (Modulo 360)
    float currentYaw = sensorHub.GetCurrentYaw();
    float yawError = targetYaw - currentYaw;

    while (yawError > 180.0f)  yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    // 6. Deadzone: voorkom dat de robot gaat "hunten" of trillen bij minuscule afwijkingen
    if (fabs(yawError) < 0.05f) {
        yawError = 0.0f;
    }

    // 7. Yaw PID berekening
    // We willen dat de error naar 0 gaat, dus we sturen op -yawError
    float steerCorrection = pIDYaw.Compute(0, -yawError); 

    // 8. Snelheid per wiel berekenen
    // SteerCorrection telt op bij het ene wiel en trekt af van het andere
    float targetLeft  = rampedLinear - steerCorrection;
    float targetRight = rampedLinear + steerCorrection;

    // 9. Wiel PID correctie (Snelheidsregeling per wiel)
    float currentLeft  = sensorHub.GetSpeedLeft();
    float currentRight = sensorHub.GetSpeedRight();

    float correctionLeft  = pIDLeft.Compute(currentLeft,  targetLeft);
    float correctionRight = pIDRight.Compute(currentRight, targetRight);

    // 10. PWM Output bepalen
    pwmLeft  = correctionLeft;
    pwmRight = correctionRight;

    // 11. Clamping & Deadband Compensation
    // We passen de minimum PWM alleen toe als we ook echt willen bewegen
    auto applyClamp = [](float& pwm, float minPwm) {
        if (abs(pwm) < 1.0f) { // Als target bijna 0 is, zet motor echt uit
            pwm = 0;
        } else {
            if (pwm > 255.0f) pwm = 255.0f;
            if (pwm < -255.0f) pwm = -255.0f; // Support voor achteruit
            
            // Zorg dat de motor altijd genoeg prik krijgt om te draaien
            if (pwm > 0 && pwm < minPwm) pwm = minPwm;
            if (pwm < 0 && pwm > -minPwm) pwm = -minPwm;
        }
    };

    applyClamp(pwmLeft, 23.0f);
    applyClamp(pwmRight, 22.5f);

    // 12. Motoren aansturen
    if (enableMotorA) motorLeft.SetSpeed(pwmLeft);
    if (enableMotorB) motorRight.SetSpeed(pwmRight);

    // Optioneel: Debugging
    // printf("TargetYaw: %.2f | Current: %.2f | Error: %.2f | Steer: %.2f\n", targetYaw, currentYaw, yawError, steerCorrection);
}
void Drive::Stop() {
    motorDirection = STOPPED;
    onTargetPos    = true;
    pwmLeft        = 50.0f;  // reset naar startwaarde
    pwmRight       = 50.0f;

    motorLeft.Stop();
    motorRight.Stop();

    pIDLeft.Reset();
    pIDRight.Reset();
}

