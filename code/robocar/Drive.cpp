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

    // Bewaar het teken, schaal alleen de magnitude
    float sign = (percent >= 0.0f) ? 1.0f : -1.0f;
    float mag  = fabs(percent);

    float pwm = minPwm + (mag / 100.0f) * (255.0f - minPwm);
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
        pIDLeft.Reset();
        pIDRight.Reset();

        // Zelfde feedforward als jouw bestaande tuning
        float targetAngVel   = fabs(angular);
        float feedforwardPwm = (targetAngVel + 52.2f) / 1.34f;
        if (feedforwardPwm < minPwmLeft) feedforwardPwm = minPwmLeft;
        if (feedforwardPwm > 255.0f)     feedforwardPwm = 255.0f;

        pwmL = feedforwardPwm;
        pwmR = feedforwardPwm;

        if (motorDirection == TURN_LEFT)
            pwmL = -pwmL;     // links achteruit, rechts vooruit -> draait links
        else
            pwmR = -pwmR;     // rechts achteruit, links vooruit -> draait rechts

        printf("[TURN] dir=%s tgt=%.1f ff=%.1f pwmL=%.1f pwmR=%.1f\n",
            motorDirection == TURN_LEFT ? "LEFT" : "RIGHT",
            targetAngVel, feedforwardPwm, pwmL, pwmR);
    }
    // ----------------------------------------
    // Rechtdoor of bocht (linear != 0)
    // ----------------------------------------
    else
    {
        // Differential drive kinematica:
        //   v_diff = omega * wheelbase / 2   (per wiel, omega in rad/s)
        // omega in graden/s -> rad/s
        float wheelbaseMm = wheelbase * 1000.0f;
        float angularRad  = angular * (M_PI / 180.0f);
        float vDiff       = angularRad * wheelbaseMm * 0.5f;  // mm/s per wiel

        // Doelsnelheden per wiel (kunnen negatief zijn bij achteruit)
        // angular > 0 = rechts -> linker wiel sneller dan rechter
        float targetLeft  = rampedLinear + vDiff;
        float targetRight = rampedLinear - vDiff;

        // Yaw-drift correctie ALLEEN bij rechtdoor rijden.
        // Bij een bocht willen we juist draaien, dus dan geen correctie.
        if (fabs(angular) < 0.001f)
        {
            float steer = ComputeSteerCorrection();
            targetLeft  -= steer;
            targetRight += steer;
        }
        else
        {
            // Bij elke bocht: yaw-tracking resetten zodat rechtdoor
            // daarna weer met een schone lei begint.
            isInitialYawSet = false;
        }

        // Encoder is altijd positief: voed PID met absolute waarden
        float absTargetL = fabs(targetLeft);
        float absTargetR = fabs(targetRight);
        float speedL     = sensorHub.GetSpeedLeft();
        float speedR     = sensorHub.GetSpeedRight();

        float outLeft  = pIDLeft .Compute(speedL, absTargetL);  // 0..100 %
        float outRight = pIDRight.Compute(speedR, absTargetR);

        // PercentToPwm levert positieve magnitude
        float magL = PercentToPwm(outLeft,  minPwmLeft);
        float magR = PercentToPwm(outRight, minPwmRight);

        // Bepaal de gewenste rij-richting per wiel.
        // Bij rechtdoor en bocht is dat het teken van linear (allebei dezelfde
        // richting). Het wiel mag NOOIT de andere kant op draaien tijdens
        // rechtdoor/bocht, anders krijg je gestotter.
        float driveSign = (linear >= 0.0f) ? 1.0f : -1.0f;

        // Magnitude mag niet negatief worden (geen sign-flip).
        // PercentToPwm geeft al >= 0, maar voor de zekerheid:
        if (magL < 0.0f) magL = 0.0f;
        if (magR < 0.0f) magR = 0.0f;

        // Stilstaande targets (mag wel echt 0 worden)
        if (absTargetL < 1.0f) { magL = 0.0f; pIDLeft.Reset();  }
        if (absTargetR < 1.0f) { magR = 0.0f; pIDRight.Reset(); }

        pwmL = driveSign * magL;
        pwmR = driveSign * magR;

        ApplyClamp(pwmL, minPwmLeft);
        ApplyClamp(pwmR, minPwmRight);

        const char* tag =
            (motorDirection == FORWARD)  ? "FWD" :
            (motorDirection == BACKWARD) ? "BWD" : "MOV";

        printf("[%s] ramp=%.1f ang=%.1f tgtL=%.1f tgtR=%.1f spdL=%.1f spdR=%.1f pwmL=%.0f pwmR=%.0f\n",
               tag, rampedLinear, angular, targetLeft, targetRight,
               speedL, speedR, pwmL, pwmR);
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
    pwmLeft        = 0.0f;
    pwmRight       = 0.0f;
    motorLeft.Stop();
    motorRight.Stop();
    pIDLeft.Reset();
    pIDRight.Reset();
    pIDYaw.Reset();
}