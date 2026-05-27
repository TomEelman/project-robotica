#include "Drive.h"
#include <cstdio>
#include <cmath>

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
//Helper
static constexpr float PID_MAX_INTEGRAL = 150.0f;
static constexpr float PID_MAX_OUTPUT   = 100.0f;

// Minimum PWM below which the motor physically does not move (stall zone).
static constexpr float CLAMP_MIN_PWM = 30.0f;

// EMA filter coefficient for encoder speed readings.
// 0.5 = equal weight to new and previous sample.
// Lower → smoother but slower to react; higher → noisier but faster.
static constexpr float EMA_ALPHA = 0.5f;

// During pure-rotation the feedforward PWM is derived from the target angular
// velocity using this linear approximation (experimentally determined):
//   pwm = (angVel_deg_s + 52.2) / 1.34
static constexpr float FF_OFFSET = 52.2f;
static constexpr float FF_SCALE  = 1.34f;

static constexpr float TURN_SPEED_GAIN          = 0.3f;
static constexpr float TURN_COUPLED_MIN_FACTOR  = 0.5f;
static constexpr float TURN_COUPLED_MAX_FACTOR  = 1.5f;

// Ramping rates in mm/s per Execute() tick.
// ACCEL and DECEL are kept equal so the robot brakes just as gradually as
// it accelerates — symmetric ramps prevent wheel slip in both directions.
// Tune these if you want sharper braking: raise DECEL toward ~10–15.
static constexpr float RAMP_ACCEL_STEP = 5.0f;
static constexpr float RAMP_DECEL_STEP = 5.0f;



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
      minAngVel(minAngVel),
      maxAngVel(maxAngVel),
      minPwmLeft(minPwmLeft),
      minPwmRight(minPwmRight)
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
        // This prevents the sudden motor reversal jerk that causes slip at
        // the entry of a turn.
        // NOTE: do NOT touch rampedTurnSpeed here — ExecuteTurn owns it.
        //       Zeroing it here (inside the turn branch) caused the ramp to
        //       reset to 0 every tick, keeping it stuck at RAMP_ACCEL_STEP.
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
    // starts fresh from zero (smooth entry).  This runs every tick while not
    // turning, so rampedTurnSpeed is guaranteed to be 0 when a new turn begins.
    rampedTurnSpeed = 0.0f;

    if (driveMode == DriveMode::Stopped) {
        rampedLinear = 0.0f;
        return;
    }

    // Linear (forward / backward) mode ─────────────────────────────────────
    float absTgt  = fabsf(linearTarget);
    float absRamp = fabsf(rampedLinear);
    float sign    = (linearTarget >= 0.0f) ? 1.0f : -1.0f;

    // Jump to minPwmLeft on the very first tick so the motor overcomes
    // static friction immediately rather than ramping through the dead zone.
    if (absRamp < minPwmLeft) absRamp = minPwmLeft;

    if (absRamp < absTgt) {
        // Accelerating — approach target at ACCEL rate.
        absRamp += RAMP_ACCEL_STEP;
        if (absRamp > absTgt) absRamp = absTgt;
    } else if (absRamp > absTgt) {
        // Decelerating — reduce at DECEL rate instead of snapping.
        // Equal rate to acceleration keeps braking as gentle as starting.
        absRamp -= RAMP_DECEL_STEP;
        if (absRamp < absTgt) absRamp = absTgt;
    }

    rampedLinear = sign * absRamp;
}

float Drive::ComputeYawCorrection()
{
    // targetYaw is latched in Execute() on the first tick of a new move,
    // so yawInitialized is already true when we get here during normal driving.
    // The guard below is a fallback for any edge case where that didn't happen.
    if (!yawInitialized) {
        targetYaw      = sensorHub.GetCurrentYaw();
        yawInitialized = true;
        pidYaw.Reset();
    }

    float currentYaw = sensorHub.GetCurrentYaw();
    float yawError   = targetYaw - currentYaw;

    // Wrap to [-180, 180] to handle the 0/360 boundary.
    while (yawError >  180.0f) yawError -= 360.0f;
    while (yawError < -180.0f) yawError += 360.0f;

    // Negate: positive yawError (drifted left) → positive output → steer right.
    return pidYaw.Compute(0.0f, -yawError);
}

void Drive::ExecuteTurn(float angular)
{
    yawInitialized = false;

    float wheelbaseMm    = wheelbaseMeters * 1000.0f;
    float angularRad     = fabsf(angular) * (static_cast<float>(M_PI) / 180.0f);
    float targetWheelSpd = angularRad * wheelbaseMm * 0.5f;   // final (full-speed) setpoint

    // ── Ramp the effective turn speed ────────────────────────────────────────
    // Instead of jumping straight to targetWheelSpd we nudge rampedTurnSpeed
    // toward it each tick.  This avoids the sudden torque spike (and resulting
    // tyre slip / encoder noise) that occurs when a pure rotation starts or
    // stops abruptly.
    if (rampedTurnSpeed < targetWheelSpd) {
        rampedTurnSpeed += RAMP_ACCEL_STEP;
        if (rampedTurnSpeed > targetWheelSpd) rampedTurnSpeed = targetWheelSpd;
    } else if (rampedTurnSpeed > targetWheelSpd) {
        rampedTurnSpeed -= RAMP_DECEL_STEP;
        if (rampedTurnSpeed < targetWheelSpd) rampedTurnSpeed = targetWheelSpd;
    }

    // Feed-forward based on the FULL requested angular velocity so the motor
    // receives enough PWM from the very first tick to overcome static friction.
    // (Using rampedTurnSpeed here made feedforward too small at startup and kept
    //  the robot stuck at minPwm even after fixing the ramp reset bug.)
    float feedforward = (fabsf(angular) + FF_OFFSET) / FF_SCALE;
    if (feedforward < minPwmLeft) feedforward = minPwmLeft;
    if (feedforward > 255.0f)     feedforward = 255.0f;

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

    // Coupled target: drive both wheels toward their current average and only
    // gently pull that average toward the ramped setpoint.
    float avgSpeed      = 0.5f * (speedLeftFiltered + speedRightFiltered);
    float avgError      = rampedTurnSpeed - avgSpeed;
    float coupledTarget = avgSpeed + TURN_SPEED_GAIN * avgError;
    if (coupledTarget < TURN_COUPLED_MIN_FACTOR * rampedTurnSpeed)
        coupledTarget = TURN_COUPLED_MIN_FACTOR * rampedTurnSpeed;
    if (coupledTarget > TURN_COUPLED_MAX_FACTOR * rampedTurnSpeed)
        coupledTarget = TURN_COUPLED_MAX_FACTOR * rampedTurnSpeed;

    // When both wheels are still at rest, reset the PID history so accumulated
    // integral from the previous linear phase does not pollute the turn startup.
    // Force output to 0 (= no trim, feedforward alone drives the motor).
    float outLeft, outRight;
    if (speedLeftFiltered < 1.0f && speedRightFiltered < 1.0f) {
        outLeft  = lastOutputLeft  = 0.0f;   // neutral trim — feedforward handles startup
        outRight = lastOutputRight = 0.0f;
        pidLeft .Reset();
        pidRight.Reset();
    } else {
        outLeft  = freshL ? pidLeft .Compute(speedLeftFiltered,  coupledTarget) : lastOutputLeft;
        outRight = freshR ? pidRight.Compute(speedRightFiltered, coupledTarget) : lastOutputRight;
        lastOutputLeft  = outLeft;
        lastOutputRight = outRight;
    }

    // PIDController::Compute returns values in [-100, 100] with 0 = on-target.
    // Map to a PWM trim in [-(255-minPwm), +(255-minPwm)].
    //
    // BUG that was here: (outLeft - 50) / 50 * 225 assumes PID center = 50.
    // Actual PID center = 0, so a healthy output of ~5 gave trim = -202,
    // collapsing magL to negative → clamped to minPwm → motor stuck at 30 PWM.
    float trimL = outLeft  / 100.0f * (255.0f - minPwmLeft);
    float trimR = outRight / 100.0f * (255.0f - minPwmRight);

    float magL = feedforward + trimL;
    float magR = feedforward + trimR;

    if (magL < minPwmLeft)  magL = minPwmLeft;
    if (magL > 255.0f)      magL = 255.0f;
    if (magR < minPwmRight) magR = minPwmRight;
    if (magR > 255.0f)      magR = 255.0f;

    if (driveMode == DriveMode::TurnLeft)  { pwmLeft = -magL; pwmRight = +magR; }
    else                                   { pwmLeft = +magL; pwmRight = -magR; }

    printf("[TURN] dir=%s tgt=%.1f ramp=%.1f coup=%.1f spdL=%.1f spdR=%.1f pwmL=%.0f pwmR=%.0f\n",
        driveMode == DriveMode::TurnLeft ? "LEFT" : "RIGHT",
        targetWheelSpd, rampedTurnSpeed, coupledTarget,
        speedLeftFiltered, speedRightFiltered, pwmLeft, pwmRight);
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

    float rawL  = sensorHub.GetSpeedLeft();
    float rawR  = sensorHub.GetSpeedRight();
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

    // Only recompute PID when the encoder produced a new sample.
    // Holding the previous output between samples avoids the 0/spike/0 pattern
    // that occurs when the integrator sees dt = 0 on a stale measurement.
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

    if (fabsf(linear) < 0.001f && fabsf(angular) < 0.001f) {
        Stop();
        return;
    }

    UpdateDriveMode(linear, angular);
    UpdateRamp(linear);

    // Latch the target yaw as early as possible — on the very first tick of a
    // new linear move, before ExecuteLinear runs. This way the yaw PID already
    // has a reference heading during the ramp phase and can suppress the small
    // left/right wobble that occurs while the encoders are still at low speed.
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