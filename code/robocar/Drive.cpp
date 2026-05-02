#include "Drive.h"
#include <cstdio>
#include <cmath>


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
      pIDLeft (0.145f, 1.763f, 0.0074f, 150.f, 100.0f),
      pIDRight(0.135f, 1.688f, 0.0074f, 150.f, 100.0f),
      pIDYaw  (4.0f,   0.1f,   0.3f,    400.f, 100.0f),
      wheelbase(Wheelbase),
      threshold(Threshold),
      enableMotorA(true),
      enableMotorB(true),
      motorDirection(STOPPED),
      pwmLeft(0.0f),
      pwmRight(0.0f),
      initialYaw(0.0f),
      targetYaw(0.0f),
      isInitialYawSet(false),
      currentAngular(0.0f),
      rampedLinear(0.0f),
      rampStep(5.0f),
      minAngVel(MinAngVel),
      maxAngVel(MaxAngVel),
      minPwmLeft(MinPwmLeft),
      minPwmRight(MinPwmRight)
{
}


// ----------------------------------------
// Helpers
// ----------------------------------------

void Drive::ApplyClamp(float& pwm, float minPwm)
{
    if (fabs(pwm) < minPwm)
    {
        pwm = 0.0f;
    }
    else
    {
        if (pwm >  255.0f) pwm =  255.0f;
        if (pwm < -255.0f) pwm = -255.0f;
    }
}

float Drive::PercentToPwm(float percent, float minPwm)
{
    if (percent < -100.0f) percent = -100.0f;
    if (percent >  100.0f) percent =  100.0f;

    float sign = (percent >= 0.0f) ? 1.0f : -1.0f;
    float mag  = fabs(percent);

    // Lineair van 0..255, geen floor
    float pwm = (mag / 100.0f) * 255.0f;
    return sign * pwm;
}

float Drive::ComputeEncoderAngVel() const
{
    // Encoder geeft altijd positief. We gebruiken hier het teken van
    // de huidige rijrichting om de rotatie te bepalen.
    float speedL = sensorHub.GetSpeedLeft();
    float speedR = sensorHub.GetSpeedRight();
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

// Ramp werkt nu op de absolute waarde van linear.
// rampedLinear heeft hetzelfde teken als linear (gewenste richting).
void Drive::UpdateRamp(float linear)
{
    if (motorDirection == TURN_LEFT || motorDirection == TURN_RIGHT
        || motorDirection == STOPPED)
    {
        rampedLinear = 0.0f;
        return;
    }

    float target  = linear;                 // bv. +378 of -378
    float absTgt  = fabs(target);
    float absRamp = fabs(rampedLinear);
    float sign    = (target >= 0.0f) ? 1.0f : -1.0f;

    // Eerste stap: spring op een minimum dat de motor uberhaupt laat draaien
    if (absRamp < minPwmLeft)
        absRamp = minPwmLeft;

    absRamp += rampStep;
    if (absRamp > absTgt) absRamp = absTgt;

    rampedLinear = sign * absRamp;
}

// Yaw-correctie alleen voor rechtdoor rijden (angular ~ 0).
// Houdt een doel-yaw vast en corrigeert drift.
float Drive::ComputeSteerCorrection()
{
    if (!isInitialYawSet)
    {
        initialYaw      = sensorHub.GetCurrentYaw();
        targetYaw       = initialYaw;
        isInitialYawSet = true;
        pIDYaw.Reset();
    }

    float currentYaw = sensorHub.GetCurrentYaw();
    float yawError   = targetYaw - currentYaw;
    while (yawError >  180.0f) yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    return pIDYaw.Compute(0.0f, -yawError);
}


// ----------------------------------------
// Execute
// ----------------------------------------
void Drive::Execute(const DriveCommand& Command)
{
    float linear  = Command.GetLinVelocity();   // mm/s, kan negatief zijn
    float angular = Command.GetAngVelocity();   // graden/s, + = rechts, - = links
    currentAngular = angular;

    // STOP
    if (fabs(linear) < 0.001f && fabs(angular) < 0.001f)
    {
        isInitialYawSet = false;
        rampedLinear    = 0.0f;
        pIDLeft.Reset();
        pIDRight.Reset();
        pIDYaw.Reset();
        lastOutLeft  = 0.0f;
        lastOutRight = 0.0f;
        filtInit     = false;
        Stop();
        return;
    }

    UpdateDirection(linear, angular);
    UpdateRamp(linear);

    float pwmL = 0.0f;
    float pwmR = 0.0f;

    // ----------------------------------------
    // Pure rotatie (linear == 0, angular != 0)
    // ----------------------------------------
    if (motorDirection == TURN_LEFT || motorDirection == TURN_RIGHT)
{
    isInitialYawSet = false;

    // ----- Doel-wielsnelheid berekenen uit gewenste yawrate -----
    // Pure rotatie: v_wiel = omega * wheelbase / 2
    // omega in graden/s -> rad/s
    float wheelbaseMm     = wheelbase * 1000.0f;
    float angularRad      = fabs(angular) * (M_PI / 180.0f);
    float targetWheelSpd  = angularRad * wheelbaseMm * 0.5f;  // mm/s, positief

    // ----- Feedforward zodat hij vanuit stilstand begint -----
    // Jouw oude formule mapt yawrate -> pwm. Hou hem als basis.
    float targetAngVel   = fabs(angular);
    float feedforwardPwm = (targetAngVel + 52.2f) / 1.34f;
    if (feedforwardPwm < minPwmLeft) feedforwardPwm = minPwmLeft;
    if (feedforwardPwm > 255.0f)     feedforwardPwm = 255.0f;

    // ----- Lees encoders met fresh-flags + EMA -----
    float rawL = sensorHub.GetSpeedLeft();
    float rawR = sensorHub.GetSpeedRight();

    bool freshL = sensorHub.HasFreshLeft();
    bool freshR = sensorHub.HasFreshRight();
    sensorHub.ConsumeFreshFlags();

    if (!filtInit)
    {
        speedLFilt = rawL;
        speedRFilt = rawR;
        filtInit   = true;
    }

    const float alpha = 0.5f;
    if (freshL) speedLFilt = alpha * rawL + (1.0f - alpha) * speedLFilt;
    if (freshR) speedRFilt = alpha * rawR + (1.0f - alpha) * speedRFilt;

    // ----- PID per wiel naar dezelfde targetWheelSpd -----
    float outLeft  = freshL ? pIDLeft .Compute(speedLFilt, targetWheelSpd) : lastOutLeft;
    float outRight = freshR ? pIDRight.Compute(speedRFilt, targetWheelSpd) : lastOutRight;
    lastOutLeft  = outLeft;
    lastOutRight = outRight;

    // PID-output (in %) is hier een TRIM bovenop feedforward, geen volledige pwm.
    // Bij output = 50% (mid-range) is trim = 0. Onder = afremmen, boven = bijgeven.
    // Schaal: 100% trim-range = ±(255-minPwm) pwm.
    float trimL = (outLeft  - 50.0f) / 50.0f * (255.0f - minPwmLeft);
    float trimR = (outRight - 50.0f) / 50.0f * (255.0f - minPwmRight);

    float magL = feedforwardPwm + trimL;
    float magR = feedforwardPwm + trimR;

    // Clamp magnitudes
    if (magL < minPwmLeft)  magL = minPwmLeft;
    if (magL > 255.0f)      magL = 255.0f;
    if (magR < minPwmRight) magR = minPwmRight;
    if (magR > 255.0f)      magR = 255.0f;

    // Tekens: TURN_LEFT = links achteruit, rechts vooruit
    if (motorDirection == TURN_LEFT)
    {
        pwmL = -magL;
        pwmR = +magR;
    }
    else // TURN_RIGHT
    {
        pwmL = +magL;
        pwmR = -magR;
    }

    printf("[TURN] dir=%s tgtSpd=%.1f spdL=%.1f spdR=%.1f outL=%.1f outR=%.1f pwmL=%.0f pwmR=%.0f\n",
        motorDirection == TURN_LEFT ? "LEFT" : "RIGHT",
        targetWheelSpd, speedLFilt, speedRFilt, outLeft, outRight, pwmL, pwmR);
}
    // ----------------------------------------
    // Rechtdoor of bocht (linear != 0)
    // ----------------------------------------
    else
    {
        float wheelbaseMm = wheelbase * 1000.0f;
        float angularRad  = angular * (M_PI / 180.0f);
        float vDiff       = angularRad * wheelbaseMm * 0.5f;

        float targetLeft  = rampedLinear + vDiff;
        float targetRight = rampedLinear - vDiff;

        // Yaw-drift correctie alleen bij echt rechtdoor rijden
        if (fabs(angular) < 0.001f)
        {
            float steer = ComputeSteerCorrection();
            targetLeft  -= steer;
            targetRight += steer;
        }
        else
        {
            isInitialYawSet = false;
        }

        // Encoder is altijd positief: PID krijgt absolute waarden
        float absTargetL = fabs(targetLeft);
        float absTargetR = fabs(targetRight);

        // ----- Lees ruwe speeds + check op verse data -----
        float rawL = sensorHub.GetSpeedLeft();
        float rawR = sensorHub.GetSpeedRight();

        bool freshL = sensorHub.HasFreshLeft();
        bool freshR = sensorHub.HasFreshRight();
        sensorHub.ConsumeFreshFlags();

        // Filter initialiseren bij eerste tick na start
        if (!filtInit)
        {
            speedLFilt = rawL;
            speedRFilt = rawR;
            filtInit   = true;
        }

        // EMA filter alleen op verse data updaten
        // alpha = 0.5 -> half oude, half nieuwe waarde. Lager = gladder, trager.
        const float alpha = 0.5f;
        if (freshL) speedLFilt = alpha * rawL + (1.0f - alpha) * speedLFilt;
        if (freshR) speedRFilt = alpha * rawR + (1.0f - alpha) * speedRFilt;

        // PID alleen herberekenen als er verse data is.
        // Tussen samples houden we de vorige output vast -> geen wind-up,
        // geen 0/-147/0 gestotter meer.
        float outLeft  = freshL ? pIDLeft .Compute(speedLFilt, absTargetL) : lastOutLeft;
        float outRight = freshR ? pIDRight.Compute(speedRFilt, absTargetR) : lastOutRight;
        lastOutLeft  = outLeft;
        lastOutRight = outRight;

        // Map % -> PWM magnitude
        float magL = PercentToPwm(outLeft,  minPwmLeft);
        float magR = PercentToPwm(outRight, minPwmRight);

        // Rij-richting per wiel = teken van linear (beide wielen zelfde kant op)
        float driveSign = (linear >= 0.0f) ? 1.0f : -1.0f;

        if (magL < 0.0f) magL = 0.0f;
        if (magR < 0.0f) magR = 0.0f;

        // Stilstaande targets mogen echt 0 worden
        if (absTargetL < 1.0f) { magL = 0.0f; pIDLeft.Reset();  lastOutLeft  = 0.0f; }
        if (absTargetR < 1.0f) { magR = 0.0f; pIDRight.Reset(); lastOutRight = 0.0f; }

        pwmL = driveSign * magL;
        pwmR = driveSign * magR;

        ApplyClamp(pwmL, 35.0f);
        ApplyClamp(pwmR, 35.0f);

        const char* tag =
            (motorDirection == FORWARD)  ? "FWD" :
            (motorDirection == BACKWARD) ? "BWD" : "MOV";

        printf("[%s] ramp=%.1f ang=%.1f tgtL=%.1f tgtR=%.1f rawL=%.1f rawR=%.1f fL=%.1f fR=%.1f frL=%d frR=%d pwmL=%.0f pwmR=%.0f\n",
               tag, rampedLinear, angular, targetLeft, targetRight,
               rawL, rawR, speedLFilt, speedRFilt,
               (int)freshL, (int)freshR, pwmL, pwmR);
    }

    pwmLeft  = pwmL;
    pwmRight = pwmR;

    if (enableMotorA) motorLeft .SetSpeed(pwmLeft);
    if (enableMotorB) motorRight.SetSpeed(pwmRight);
}

// ----------------------------------------
// Stop
// ----------------------------------------

void Drive::Stop()
{
    motorDirection = STOPPED;
    pwmLeft  = 0.0f;
    pwmRight = 0.0f;
    motorLeft.Stop();
    motorRight.Stop();
    pIDLeft.Reset();
    pIDRight.Reset();
    pIDYaw.Reset();
    lastOutLeft  = 0.0f;
    lastOutRight = 0.0f;
    filtInit = false;     // filter opnieuw initialiseren bij volgende start
}