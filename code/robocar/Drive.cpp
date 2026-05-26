#include "Drive.h"
#include <cstdio>
#include <cmath>

// Left motor requires slightly different PID tuning than right motor
// Left PID
static constexpr float PID_LEFT_KP  = 0.145f;
static constexpr float PID_LEFT_KI  = 1.763f;
static constexpr float PID_LEFT_KD  = 0.0074f;

// Right PID
static constexpr float PID_RIGHT_KP = 0.135f;
static constexpr float PID_RIGHT_KI = 1.688f;
static constexpr float PID_RIGHT_KD = 0.0074f;

// Yaw PID
static constexpr float PID_YAW_KP   = 4.0f;
static constexpr float PID_YAW_KI   = 0.1f;
static constexpr float PID_YAW_KD   = 0.3f;

// Helper
static constexpr float PID_MAX_INTEGRAL = 150.0f;
static constexpr float PID_MAX_OUTPUT   = 100.0f;

// Minimum PWM below which the motor physically does not move (stall zone).
static constexpr float CLAMP_MIN_PWM = 30.0f;

// EMA filter coefficient for encoder speed readings.
// 0.5 = equal weight to new and previous sample.
// Lower → smoother but slower to react; higher → noisier but faster.
static constexpr float EMA_ALPHA = 0.5f;

// During only rotation feedforward PWM is derived from the target angular
// velocity using this linear approximation:
// pwm = (angVel_deg_s + 52.2) / 1.34
static constexpr float FF_OFFSET = 52.2f;
static constexpr float FF_SCALE  = 1.34f;

static constexpr float TURN_SPEED_GAIN         = 0.3f;
static constexpr float TURN_COUPLED_MIN_FACTOR = 0.5f;
static constexpr float TURN_COUPLED_MAX_FACTOR = 1.5f;

static constexpr float DEFAULT_RAMP_STEP = 5.0f; // mm/s per tick for ramped linear

// OutputLimiter: maximum PWM change per Execute() tick.
// At 100 ms loop time, PWM_MAX_STEP_UP = 12 means ~120 PWM/s acceleration.
// Deceleration is allowed to be faster (asymmetric) to stop quickly when needed.
static constexpr float PWM_MAX_STEP_UP   = 12.0f;
static constexpr float PWM_MAX_STEP_DOWN = 25.0f;


// ─────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────

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
      lastLimitedPwmLeft(0.0f),
      lastLimitedPwmRight(0.0f),
      targetYaw(0.0f),
      yawInitialized(false),
      rampedLinear(0.0f),
      rampStep(DEFAULT_RAMP_STEP),
      minAngVel(minAngVel),
      maxAngVel(maxAngVel),
      minPwmLeft(minPwmLeft),
      minPwmRight(minPwmRight)
{
}


// ─────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────

void Drive::clampPwm(float& pwm, float minPwm)
{
    if (fabsf(pwm) < minPwm)
        pwm = 0.0f;
    else if (pwm >  255.0f)
        pwm =  255.0f;
    else if (pwm < -255.0f)
        pwm = -255.0f;
}

float Drive::percentToPwm(float percent)
{
    if (percent < -100.0f) percent = -100.0f;
    if (percent >  100.0f) percent =  100.0f;
    return (percent / 100.0f) * 255.0f;
}

// applyOutputLimiter — limits the PWM delta per tick for one channel.
//
// This is the last line of defense against slip-inducing PWM jumps,
// regardless of whether the jump originated from the ramp, the PID,
// or a direct command from Navigator / obstacle avoidance.
//
// Acceleration (same sign, growing magnitude) is limited to PWM_MAX_STEP_UP.
// Deceleration (shrinking magnitude) is limited to PWM_MAX_STEP_DOWN.
// Direction reversal passes through zero at PWM_MAX_STEP_DOWN per tick.
float Drive::applyOutputLimiter(float target, float previous)
{
    float delta = target - previous;

    // Clamp upward step (acceleration)
    if (delta >  PWM_MAX_STEP_UP)   delta =  PWM_MAX_STEP_UP;
    // Clamp downward step (deceleration / reversal)
    if (delta < -PWM_MAX_STEP_DOWN) delta = -PWM_MAX_STEP_DOWN;

    return previous + delta;
}

void Drive::updateDriveMode(float linear, float angular)
{
    const float LIN_DEAD = 0.001f;
    const float ANG_DEAD = 0.001f;

    if (fabsf(linear) < LIN_DEAD && angular >  ANG_DEAD) {
        driveMode = DriveMode::TurnRight;
    } else if (fabsf(linear) < LIN_DEAD && angular < -ANG_DEAD) {
        driveMode = DriveMode::TurnLeft;
    } else if (linear >  LIN_DEAD) {
        driveMode = DriveMode::Forward;
    } else if (linear < -LIN_DEAD) {
        driveMode = DriveMode::Backward;
    } else {
        driveMode = DriveMode::Stopped;
    }
}

void Drive::updateRamp(float linearTarget)
{
    if (driveMode == DriveMode::TurnLeft  ||
        driveMode == DriveMode::TurnRight ||
        driveMode == DriveMode::Stopped)
    {
        // During turns and stops the linear ramp is zeroed so that when linear
        // motion resumes the ramp starts cleanly from zero again.
        rampedLinear = 0.0f;
        return;
    }

    float absTgt  = fabsf(linearTarget);
    float absRamp = fabsf(rampedLinear);
    float sign    = (linearTarget >= 0.0f) ? 1.0f : -1.0f;

    // Jump over stall zone: if the ramp is below the minimum PWM the motor
    // would not move at all, so skip straight to minPwmLeft.
    if (absRamp < minPwmLeft) {
        absRamp = minPwmLeft;
    }

    absRamp += rampStep;

    if (absRamp > absTgt) {
        absRamp = absTgt;
    }

    rampedLinear = sign * absRamp;
}

float Drive::computeYawCorrection()
{
    // Fallback: normally yawInitialized is set in Execute() before this call.
    if (!yawInitialized) {
        targetYaw      = sensorHub.GetCurrentYaw();
        yawInitialized = true;
        pidYaw.Reset();
    }

    float currentYaw = sensorHub.GetCurrentYaw();
    float yawError   = targetYaw - currentYaw;

    // Wrap to (-180, 180]
    while (yawError >  180.0f) {
        yawError -= 360.0f;
    }
    while (yawError < -180.0f) {
        yawError += 360.0f;
    }

    return pidYaw.Compute(0.0f, -yawError);
}


// ─────────────────────────────────────────────────────────────────
// executeTurn — pure in-place rotation
// ─────────────────────────────────────────────────────────────────

void Drive::executeTurn(float angular)
{
    yawInitialized = false;

    float wheelbaseMm    = wheelbaseMeters * 1000.0f;
    float angularRad     = fabsf(angular) * (M_PI / 180.0f);
    float targetWheelSpd = angularRad * wheelbaseMm * 0.5f;

    float feedforward = (fabsf(angular) + FF_OFFSET) / FF_SCALE;
    if (feedforward < minPwmLeft) {
        feedforward = minPwmLeft;
    }

    if (feedforward > 255.0f) {
        feedforward = 255.0f;
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

    if (freshL) {
        speedLeftFiltered  = EMA_ALPHA * rawL + (1.0f - EMA_ALPHA) * speedLeftFiltered;
    }

    if (freshR) {
        speedRightFiltered = EMA_ALPHA * rawR + (1.0f - EMA_ALPHA) * speedRightFiltered;
    }

    // Coupled target: drive both wheels toward their current average and only
    // gently pull that average toward the true setpoint. This synchronises
    // the wheels without a separate balance controller.
    float avgSpeed      = 0.5f * (speedLeftFiltered + speedRightFiltered);
    float avgError      = targetWheelSpd - avgSpeed;
    float coupledTarget = avgSpeed + TURN_SPEED_GAIN * avgError;

    if (coupledTarget < TURN_COUPLED_MIN_FACTOR * targetWheelSpd) {
        coupledTarget = TURN_COUPLED_MIN_FACTOR * targetWheelSpd;
    }
    
    if (coupledTarget > TURN_COUPLED_MAX_FACTOR * targetWheelSpd) {
        coupledTarget = TURN_COUPLED_MAX_FACTOR * targetWheelSpd;
    }

    float outLeft  = freshL ? pidLeft .Compute(speedLeftFiltered,  coupledTarget) : lastOutputLeft;
    float outRight = freshR ? pidRight.Compute(speedRightFiltered, coupledTarget) : lastOutputRight;
    lastOutputLeft  = outLeft;
    lastOutputRight = outRight;

    float trimL = (outLeft  - 50.0f) / 50.0f * (255.0f - minPwmLeft);
    float trimR = (outRight - 50.0f) / 50.0f * (255.0f - minPwmRight);

    float magL = feedforward + trimL;
    float magR = feedforward + trimR;

    if (magL < minPwmLeft) {
        magL = minPwmLeft;
    }

    if (magL > 255.0f) {
        magL = 255.0f;
    } 

    if (magR < minPwmRight) {
        magR = minPwmRight;
    }

    if (magR > 255.0f) {
        magR = 255.0f;
    }

    if (driveMode == DriveMode::TurnLeft) {
        pwmLeft = -magL; pwmRight = +magR;
    } else {
        pwmLeft = +magL; pwmRight = -magR;
    }

    printf("[TURN] dir=%s tgt=%.1f coup=%.1f spdL=%.1f spdR=%.1f outL=%.0f outR=%.0f pwmL=%.0f pwmR=%.0f\n",
        driveMode == DriveMode::TurnLeft ? "LEFT" : "RIGHT",
        targetWheelSpd, coupledTarget,
        speedLeftFiltered, speedRightFiltered,
        outLeft, outRight, pwmLeft, pwmRight);
}


// ─────────────────────────────────────────────────────────────────
// executeLinear — forward / backward, with optional gentle steering
// ─────────────────────────────────────────────────────────────────

void Drive::executeLinear(float linear, float angular)
{
    float wheelbaseMm = wheelbaseMeters * 1000.0f;
    float angularRad  = angular * (static_cast<float>(M_PI) / 180.0f);
    float vDiff       = angularRad * wheelbaseMm * 0.5f;

    float targetLeft  = rampedLinear + vDiff;
    float targetRight = rampedLinear - vDiff;

    if (fabsf(angular) < 0.001f) {
        float steer  = computeYawCorrection();
        targetLeft  -= steer;
        targetRight += steer;
    } else {
        // Steering is commanded externally — release yaw hold so the PID
        // does not fight the intentional turn.
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

    if (freshL) {
        speedLeftFiltered  = EMA_ALPHA * rawL + (1.0f - EMA_ALPHA) * speedLeftFiltered;
    }

    if (freshR) {
        speedRightFiltered = EMA_ALPHA * rawR + (1.0f - EMA_ALPHA) * speedRightFiltered;
    }

    // Only recompute PID when the encoder produced a new sample.
    // Holding the previous output between samples avoids the 0/spike/0 pattern
    // that occurs when the integrator sees dt = 0 on a stale measurement.
    float outLeft  = freshL ? pidLeft .Compute(speedLeftFiltered,  absTargetL) : lastOutputLeft;
    float outRight = freshR ? pidRight.Compute(speedRightFiltered, absTargetR) : lastOutputRight;
    lastOutputLeft  = outLeft;
    lastOutputRight = outRight;

    float magL = percentToPwm(outLeft);
    float magR = percentToPwm(outRight);

    if (magL < 0.0f) {
        magL = 0.0f;
    }

    if (magR < 0.0f) {
        magR = 0.0f;
    }

    if (absTargetL < 1.0f) { 
        magL = 0.0f; pidLeft.Reset();  lastOutputLeft  = 0.0f; 
    }

    if (absTargetR < 1.0f) { 
        magR = 0.0f; pidRight.Reset(); lastOutputRight = 0.0f; 
    }

    float driveSign = (linear >= 0.0f) ? 1.0f : -1.0f;
    pwmLeft  = driveSign * magL;
    pwmRight = driveSign * magR;

    clampPwm(pwmLeft,  CLAMP_MIN_PWM);
    clampPwm(pwmRight, CLAMP_MIN_PWM);

    const char* tag =
        (driveMode == DriveMode::Forward)  ? "FWD" :
        (driveMode == DriveMode::Backward) ? "BWD" : "MOV";

    printf("[%s] ramp=%.1f ang=%.1f tgtL=%.1f tgtR=%.1f rawL=%.1f rawR=%.1f fL=%.1f fR=%.1f frL=%d frR=%d pwmL=%.0f pwmR=%.0f yaw=%.6f\n",
           tag, rampedLinear, angular, targetLeft, targetRight,
           rawL, rawR, speedLeftFiltered, speedRightFiltered,
           (int)freshL, (int)freshR, pwmLeft, pwmRight,
           sensorHub.GetCurrentYaw());
}


// ─────────────────────────────────────────────────────────────────
// Execute — public entry point
// ─────────────────────────────────────────────────────────────────

void Drive::Execute(const DriveCommand& command)
{
    float linear  = command.GetLinVelocity();
    float angular = command.GetAngVelocity();

    if (fabsf(linear) < 0.001f && fabsf(angular) < 0.001f) {
        Stop();
        return;
    }

    updateDriveMode(linear, angular);
    updateRamp(linear);

    // Capture the target yaw as soon as straight-line motion begins so the
    // yaw PID has a reference from the first tick — reduces early wobble.
    if (!yawInitialized &&
        driveMode != DriveMode::TurnLeft  &&
        driveMode != DriveMode::TurnRight &&
        driveMode != DriveMode::Stopped   &&
        fabsf(angular) < 0.001f) 
    {
        targetYaw      = sensorHub.GetCurrentYaw();
        yawInitialized = true;
        pidYaw.Reset();
    }

    if (driveMode == DriveMode::TurnLeft || driveMode == DriveMode::TurnRight) {
        executeTurn(angular);
    }
    else {
        executeLinear(linear, angular);
    }

    // OutputLimiter: applied after all inner calculations so it catches jumps
    // from any source (ramp, PID, yaw correction, feedforward).
    // lastLimitedPwm* is kept as member state so the limit is continuous
    // across ticks, not just within a single Execute() call.
    pwmLeft  = applyOutputLimiter(pwmLeft,  lastLimitedPwmLeft);
    pwmRight = applyOutputLimiter(pwmRight, lastLimitedPwmRight);
    lastLimitedPwmLeft  = pwmLeft;
    lastLimitedPwmRight = pwmRight;

    if (enableMotorLeft) {
        motorLeft .SetSpeed(pwmLeft);
    }

    if (enableMotorRight) {
        motorRight.SetSpeed(pwmRight);
    }
}


// ─────────────────────────────────────────────────────────────────
// Stop
// ─────────────────────────────────────────────────────────────────

void Drive::Stop()
{
    driveMode         = DriveMode::Stopped;
    rampedLinear      = 0.0f;
    yawInitialized    = false;
    filterInitialized = false;
    pwmLeft           = 0.0f;
    pwmRight          = 0.0f;
    lastOutputLeft    = 0.0f;
    lastOutputRight   = 0.0f;

    // Reset the limiter history so the robot can accelerate cleanly after
    // a full stop without fighting the previous PWM level.
    lastLimitedPwmLeft  = 0.0f;
    lastLimitedPwmRight = 0.0f;

    pidLeft.Reset();
    pidRight.Reset();
    pidYaw.Reset();

    motorLeft.Stop();
    motorRight.Stop();
}