#include "Drive.h"
#include <cstdio>
#include <cmath>
#include "pico/time.h"

//links klein verschil mer rechts
//Left PID
static constexpr float PID_LEFT_KP  = 0.145f;
static constexpr float PID_LEFT_KI  = 1.763f;
static constexpr float PID_LEFT_KD  = 0.0074f;

//Right PID
static constexpr float PID_RIGHT_KP = 0.135f;
static constexpr float PID_RIGHT_KI = 1.688f;
static constexpr float PID_RIGHT_KD = 0.0074f;

//yaw PID
static constexpr float PID_YAW_KP   = 4.0f;
static constexpr float PID_YAW_KI   = 0.1f;
static constexpr float PID_YAW_KD   = 0.3f;

static constexpr float PID_MAX_INTEGRAL = 49.0f;
static constexpr float PID_MAX_OUTPUT   = 100.0f;

// Minimum PWM below which the motor physically does not move (stall zone).
static constexpr float CLAMP_MIN_PWM = 30.0f;

// EMA filter coefficient for encoder speed readings.
static constexpr float EMA_ALPHA = 0.5f;

// Feedforward for pure-rotation:  pwm = (angVel_deg_s + FF_OFFSET) / FF_SCALE
static constexpr float FF_OFFSET = 52.2f;
static constexpr float FF_SCALE  = 1.34f;

static constexpr float TURN_SPEED_GAIN         = 0.3f;
static constexpr float TURN_COUPLED_MIN_FACTOR = 0.5f;
static constexpr float TURN_COUPLED_MAX_FACTOR = 1.5f;

// Ramping rates in mm/s per Execute() tick.
static constexpr float RAMP_ACCEL_STEP = 5.0f;
static constexpr float RAMP_DECEL_STEP = 5.0f;

// Maximum change in the incoming linear command per Execute() call.
static constexpr float CMD_MAX_LIN_STEP = 20.0f;

// Reversal lockout thresholds.
// The robot is considered stationary when both encoder speeds are below
// REVERSAL_STILL_MM_S. REVERSAL_TIMEOUT_MS is a safety cap so the lockout
// never hangs indefinitely (e.g. if encoder noise keeps the reading above
// the threshold).
static constexpr float    REVERSAL_STILL_MM_S  = 15.0f;
static constexpr uint32_t REVERSAL_TIMEOUT_MS  = 800;


Drive::Drive(Motor&     leftMotor,
             Motor&     rightMotor,
             SensorHub& sensors,
             float      wheelbaseMeters,
             float      minAngVel,
             float      maxAngVel,
             float      minPwmLeft,
             float      minPwmRight)
    : motorLeft(leftMotor),
      motorRight(rightMotor),
      sensorHub(sensors),
      pidLeft (PID_LEFT_KP,  PID_LEFT_KI,  PID_LEFT_KD,  PID_MAX_INTEGRAL, PID_MAX_OUTPUT),
      pidRight(PID_RIGHT_KP, PID_RIGHT_KI, PID_RIGHT_KD, PID_MAX_INTEGRAL, PID_MAX_OUTPUT),
      pidYaw  (PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,   400.0f,           PID_MAX_OUTPUT),
      wheelbaseMeters(wheelbaseMeters),
      enableMotorLeft(true),
      enableMotorRight(true),
      driveMode(DriveMode::Stopped),
      speedLeftFiltered(0.0f),
      speedRightFiltered(0.0f),
      lastOutputLeft(0.0f),
      lastOutputRight(0.0f),
      filterInitialized(false),
      pwmLeft(0.0f),
      pwmRight(0.0f),
      targetYaw(0.0f),
      yawInitialized(false),
      rampedLinear(0.0f),
      rampedTurnSpeed(0.0f),
      cmdSmoothedLinear(0.0f),
      minAngVel(minAngVel),
      maxAngVel(maxAngVel),
      minPwmLeft(minPwmLeft),
      minPwmRight(minPwmRight),
      reversalLockout(false),
      reversalStartMs(0)
{
}

void Drive::ClampPwm(float& pwm, float minPwm)
{
    if (fabsf(pwm) < minPwm)
        pwm = 0.0f;
    else if (pwm >  255.0f)
        pwm =  255.0f;
    else if (pwm < -255.0f)
        pwm = -255.0f;
}

float Drive::PercentToPwm(float percent)
{
    if (percent < -100.0f) percent = -100.0f;
    if (percent >  100.0f) percent =  100.0f;
    return (percent / 100.0f) * 255.0f;
}

void Drive::UpdateDriveMode(float linear, float angular)
{
    const float LIN_DEAD = 0.001f;
    const float ANG_DEAD = 0.001f;

    if      (fabsf(linear) < LIN_DEAD && angular >  ANG_DEAD) driveMode = DriveMode::TurnRight;
    else if (fabsf(linear) < LIN_DEAD && angular < -ANG_DEAD) driveMode = DriveMode::TurnLeft;
    else if (linear >  LIN_DEAD)                               driveMode = DriveMode::Forward;
    else if (linear < -LIN_DEAD)                               driveMode = DriveMode::Backward;
    else                                                        driveMode = DriveMode::Stopped;
}

void Drive::UpdateRamp(float linearTarget)
{
    if (driveMode == DriveMode::TurnLeft || driveMode == DriveMode::TurnRight) {
        // Ramp linear speed down to zero instead of snapping instantly.
        // NOTE: do NOT touch rampedTurnSpeed here — ExecuteTurn owns it.
        float absRamp = fabsf(rampedLinear);
        if (absRamp > 0.0f) {
            absRamp -= RAMP_DECEL_STEP;
            if (absRamp < 0.0f) absRamp = 0.0f;
        }
        float sign   = (rampedLinear >= 0.0f) ? 1.0f : -1.0f;
        rampedLinear = sign * absRamp;
        return;
    }

    // Stopped or linear mode — reset the turn ramp so the NEXT turn always
    // starts fresh from zero (smooth entry).
    rampedTurnSpeed = 0.0f;

    if (driveMode == DriveMode::Stopped) {
        float absRamp = fabsf(rampedLinear);
        if (absRamp > RAMP_DECEL_STEP) {
            float sign   = (rampedLinear >= 0.0f) ? 1.0f : -1.0f;
            rampedLinear = sign * (absRamp - RAMP_DECEL_STEP);
        } else {
            rampedLinear = 0.0f;
        }
        return;
    }

    // Linear (forward / backward) mode
    float absTgt  = fabsf(linearTarget);
    float absRamp = fabsf(rampedLinear);
    float sign    = (linearTarget >= 0.0f) ? 1.0f : -1.0f;

    // Jump to minPwmLeft on the very first tick so the motor overcomes
    // static friction immediately rather than ramping through the dead zone.
    if (absRamp < minPwmLeft) absRamp = minPwmLeft;

    if (absRamp < absTgt) {
        absRamp += RAMP_ACCEL_STEP;
        if (absRamp > absTgt) absRamp = absTgt;
    } else if (absRamp > absTgt) {
        absRamp -= RAMP_DECEL_STEP;
        if (absRamp < absTgt) absRamp = absTgt;
    }

    rampedLinear = sign * absRamp;
}

float Drive::ComputeYawCorrection()
{
    if (!yawInitialized) {
        targetYaw      = sensorHub.GetCurrentYaw();
        yawInitialized = true;
        pidYaw.Reset();
    }

    float currentYaw = sensorHub.GetCurrentYaw();
    float yawError   = targetYaw - currentYaw;

    yawError = fmodf(yawError + 180.0f, 360.0f);
    if (yawError < 0.0f) {
        yawError += 360.0f;
    }
    yawError -= 180.0f;

    return pidYaw.Compute(0.0f, -yawError);
}

void Drive::ExecuteTurn(float angular)
{
    yawInitialized = false;

    float wheelbaseMm    = wheelbaseMeters * 1000.0f;
    float angularRad     = fabsf(angular) * (static_cast<float>(M_PI) / 180.0f);
    float targetWheelSpd = angularRad * wheelbaseMm * 0.5f;

    if (rampedTurnSpeed < targetWheelSpd) {
        rampedTurnSpeed += RAMP_ACCEL_STEP;
        if (rampedTurnSpeed > targetWheelSpd) rampedTurnSpeed = targetWheelSpd;
    } else if (rampedTurnSpeed > targetWheelSpd) {
        rampedTurnSpeed -= RAMP_DECEL_STEP;
        if (rampedTurnSpeed < targetWheelSpd) rampedTurnSpeed = targetWheelSpd;
    }

    float feedforward = (fabsf(angular) + FF_OFFSET) / FF_SCALE;
    if (feedforward < minPwmLeft) feedforward = minPwmLeft;
    if (feedforward > 255.0f)     feedforward = 255.0f;

    if (speedLeftFiltered < 5.0f && speedRightFiltered < 5.0f) {
    if (feedforward < 120.0f) feedforward = 120.0f;
}

    float rawL   = sensorHub.GetSpeedLeft();
    float rawR   = sensorHub.GetSpeedRight();
    bool  freshL = sensorHub.HasFreshLeft();
    bool  freshR = sensorHub.HasFreshRight();
    sensorHub.ConsumeFreshFlags();

    if (!filterInitialized) {
        speedLeftFiltered  = rawL;
        speedRightFiltered = rawR;
        filterInitialized  = true;
    }
    if (freshL) speedLeftFiltered  = EMA_ALPHA * rawL + (1.0f - EMA_ALPHA) * speedLeftFiltered;
    if (freshR) speedRightFiltered = EMA_ALPHA * rawR + (1.0f - EMA_ALPHA) * speedRightFiltered;

    float avgSpeed      = 0.5f * (speedLeftFiltered + speedRightFiltered);
    float avgError      = rampedTurnSpeed - avgSpeed;
    float coupledTarget = avgSpeed + TURN_SPEED_GAIN * avgError;
    if (coupledTarget < TURN_COUPLED_MIN_FACTOR * rampedTurnSpeed)
        coupledTarget = TURN_COUPLED_MIN_FACTOR * rampedTurnSpeed;
    if (coupledTarget > TURN_COUPLED_MAX_FACTOR * rampedTurnSpeed)
        coupledTarget = TURN_COUPLED_MAX_FACTOR * rampedTurnSpeed;

    float outLeft;
    float outRight;
    if (speedLeftFiltered < 1.0f && speedRightFiltered < 1.0f) {
        outLeft  = lastOutputLeft  = 0.0f;
        outRight = lastOutputRight = 0.0f;
        pidLeft .Reset();
        pidRight.Reset();
    } else {
        outLeft  = freshL ? pidLeft .Compute(speedLeftFiltered,  coupledTarget) : lastOutputLeft;
        outRight = freshR ? pidRight.Compute(speedRightFiltered, coupledTarget) : lastOutputRight;
        lastOutputLeft  = outLeft;
        lastOutputRight = outRight;
    }

    float trimL = outLeft  / 100.0f * (255.0f - minPwmLeft);
    float trimR = outRight / 100.0f * (255.0f - minPwmRight);

    float magL = feedforward + trimL;
    float magR = feedforward + trimR;

    if (magL < minPwmLeft)  magL = minPwmLeft;
    if (magL > 255.0f)      magL = 255.0f;
    if (magR < minPwmRight) magR = minPwmRight;
    if (magR > 255.0f)      magR = 255.0f;

    if (driveMode == DriveMode::TurnLeft)  {
        pwmLeft = -magL; pwmRight = +magR;
    } else {
        pwmLeft = +magL; pwmRight = -magR;
    }
}

void Drive::ExecuteLinear(float linear, float angular)
{
    float wheelbaseMm = wheelbaseMeters * 1000.0f;
    float angularRad  = angular * (static_cast<float>(M_PI) / 180.0f);
    float vDiff       = angularRad * wheelbaseMm * 0.5f;

    float targetLeft  = rampedLinear + vDiff;
    float targetRight = rampedLinear - vDiff;

    if (fabsf(angular) < 0.001f) {
        float steer  = ComputeYawCorrection();
        targetLeft  -= steer;
        targetRight += steer;
    } else {
        yawInitialized = false;
    }

    float absTargetL = fabsf(targetLeft);
    float absTargetR = fabsf(targetRight);

    float rawL   = sensorHub.GetSpeedLeft();
    float rawR   = sensorHub.GetSpeedRight();
    bool  freshL = sensorHub.HasFreshLeft();
    bool  freshR = sensorHub.HasFreshRight();
    sensorHub.ConsumeFreshFlags();

    if (!filterInitialized) {
        speedLeftFiltered  = rawL;
        speedRightFiltered = rawR;
        filterInitialized  = true;
    }
    if (freshL) speedLeftFiltered  = EMA_ALPHA * rawL + (1.0f - EMA_ALPHA) * speedLeftFiltered;
    if (freshR) speedRightFiltered = EMA_ALPHA * rawR + (1.0f - EMA_ALPHA) * speedRightFiltered;

    float outLeft  = freshL ? pidLeft .Compute(speedLeftFiltered,  absTargetL) : lastOutputLeft;
    float outRight = freshR ? pidRight.Compute(speedRightFiltered, absTargetR) : lastOutputRight;
    lastOutputLeft  = outLeft;
    lastOutputRight = outRight;

    float magL = PercentToPwm(outLeft);
    float magR = PercentToPwm(outRight);

    if (magL < 0.0f) magL = 0.0f;
    if (magR < 0.0f) magR = 0.0f;

    if (absTargetL < 1.0f) { magL = 0.0f; pidLeft.Reset();  lastOutputLeft  = 0.0f; }
    if (absTargetR < 1.0f) { magR = 0.0f; pidRight.Reset(); lastOutputRight = 0.0f; }

    float driveSign = (linear >= 0.0f) ? 1.0f : -1.0f;
    pwmLeft  = driveSign * magL;
    pwmRight = driveSign * magR;

    ClampPwm(pwmLeft,  CLAMP_MIN_PWM);
    ClampPwm(pwmRight, CLAMP_MIN_PWM);

    const char* tag =
        (driveMode == DriveMode::Forward)  ? "FWD" :
        (driveMode == DriveMode::Backward) ? "BWD" : "MOV";

    printf("[%s] ramp=%.1f ang=%.1f tgtL=%.1f tgtR=%.1f rawL=%.1f rawR=%.1f fL=%.1f fR=%.1f frL=%d frR=%d pwmL=%.0f pwmR=%.0f yaw=%.6f\n",
           tag, rampedLinear, angular, targetLeft, targetRight,
           rawL, rawR, speedLeftFiltered, speedRightFiltered,
           (int)freshL, (int)freshR, pwmLeft, pwmRight,
           sensorHub.GetCurrentYaw());
}

void Drive::Execute(const DriveCommand& command)
{
    float linear  = command.GetLinVelocity();
    float angular = command.GetAngVelocity();

    printf("[EXEC] lin=%.1f ang=%.1f mode=%d lock=%d spdL=%.1f spdR=%.1f pwmL=%.0f pwmR=%.0f\n",
           linear, angular, (int)driveMode, (int)reversalLockout,
           sensorHub.GetSpeedLeft(), sensorHub.GetSpeedRight(), pwmLeft, pwmRight);
    // ── Reversal lockout ────────────────────────────────────────────────────
    // Detect a Forward→Backward or Backward→Forward transition BEFORE the
    // command rate-limiter runs (so driveMode still reflects the previous
    // physical direction). When a reversal is detected, hold the motors at
    // zero and wait until both encoder speeds fall below REVERSAL_STILL_MM_S.
    // This prevents EncoderMetTeken on the Pi5 from assigning the wrong sign
    // to the measured wheel speed while the robot is still rolling in the old
    // direction, which would cause a localisation position jump and map smear.
    //
    // Non-blocking: Execute() keeps being called every tick; we just hold Stop
    // and return early until the encoders confirm the robot has stopped.
    //
    // The lockout is based on driveMode (physical state) not on the command,
    // so keepalive repetitions of the same command do not re-trigger it.
    float linForModeCheck = linear;
    {
        // Apply only the sign part of the rate-limiter to decide if a reversal
        // is happening — we need the intended direction, not the smoothed one.
        // (The full rate-limiter runs further below, after the lockout check.)
        bool newForward  = linForModeCheck >  1.0f;
        bool newBackward = linForModeCheck < -1.0f;
        bool reversal = (newForward  && driveMode == DriveMode::Backward) ||
                        (newBackward && driveMode == DriveMode::Forward);

        if (reversal && !reversalLockout) {
            reversalLockout  = true;
            reversalStartMs  = to_ms_since_boot(get_absolute_time());
        }
    }

    if (reversalLockout) {
        float spdL   = fabsf(sensorHub.GetSpeedLeft());
        float spdR   = fabsf(sensorHub.GetSpeedRight());
        bool  stil   = (spdL < REVERSAL_STILL_MM_S && spdR < REVERSAL_STILL_MM_S);
        bool  timeout = (to_ms_since_boot(get_absolute_time()) - reversalStartMs)
                        > REVERSAL_TIMEOUT_MS;

        if (stil || timeout) {
            // Wheels are stationary (or safety timeout expired).
            // Clear the lockout and fall through to execute the new command.
            reversalLockout = false;
        } else {
            // Still coasting — keep motors at zero and return early.
            // Stop() is called to hold all ramps and PID at zero so the next
            // direction starts cleanly from a known state.
            Stop();
            return;
        }
    }

    // ── Command rate limiter ────────────────────────────────────────────────
    if (fabsf(linear) < 0.001f) {
        float delta = -cmdSmoothedLinear;
        if (delta < -CMD_MAX_LIN_STEP) delta = -CMD_MAX_LIN_STEP;
        if (delta >  CMD_MAX_LIN_STEP) delta =  CMD_MAX_LIN_STEP;
        cmdSmoothedLinear += delta;
        linear = 0.0f;
    } else {
        float linDelta = linear - cmdSmoothedLinear;
        if (linDelta >  CMD_MAX_LIN_STEP) linDelta =  CMD_MAX_LIN_STEP;
        if (linDelta < -CMD_MAX_LIN_STEP) linDelta = -CMD_MAX_LIN_STEP;
        cmdSmoothedLinear += linDelta;
        linear = cmdSmoothedLinear;
    }

    if (fabsf(linear) < 0.001f && fabsf(angular) < 0.001f) {
        Stop();
        return;
    }

    UpdateDriveMode(linear, angular);
    UpdateRamp(linear);

    if (!yawInitialized &&
        driveMode != DriveMode::TurnLeft &&
        driveMode != DriveMode::TurnRight &&
        driveMode != DriveMode::Stopped &&
        fabsf(angular) < 0.001f)
    {
        targetYaw        = sensorHub.GetCurrentYaw();
        yawInitialized   = true;
        pidYaw.Reset();
    }

    if (driveMode == DriveMode::TurnLeft || driveMode == DriveMode::TurnRight)
        ExecuteTurn(angular);
    else
        ExecuteLinear(linear, angular);

    if (enableMotorLeft)  motorLeft .SetSpeed(pwmLeft);
    if (enableMotorRight) motorRight.SetSpeed(pwmRight);
}

void Drive::Stop()
{
    driveMode         = DriveMode::Stopped;
    rampedLinear      = 0.0f;
    rampedTurnSpeed   = 0.0f;
    cmdSmoothedLinear = 0.0f;
    yawInitialized    = false;
    filterInitialized = false;
    pwmLeft           = 0.0f;
    pwmRight          = 0.0f;
    lastOutputLeft    = 0.0f;
    lastOutputRight   = 0.0f;

    pidLeft.Reset();
    pidRight.Reset();
    pidYaw.Reset();

    motorLeft.Stop();
    motorRight.Stop();
}