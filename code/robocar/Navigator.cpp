#include "Navigator.h"
#include <cmath>
#include <cstdio>

// NormDeg — normalize angle to (-180, 180]
float NormDeg(float deg) {
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}


// ─────────────────────────────────────────────────────────────────
// AnalyzeScan — sector analysis of a 360° scan (obstacle avoidance)
// ─────────────────────────────────────────────────────────────────

static float g_prevAvoid = 0.0f;

ScanAnalysis AnalyzeScan(const float ranges[360]) {
    static constexpr float CRITICAL_MM = 350.0f;
    static constexpr float BRAKE_MM    = 500.0f;
    static constexpr float SAFE_MM     = 700.0f;
    static constexpr float CHASSIS_MM  = 280.0f;
    static constexpr float MAX_RANGE   = 8000.0f;
    static constexpr float HYSTERESIS  = 200.0f;

    // Split the 360 readings into 36 sectors of 10° and keep the nearest
    // reading per sector (= the closest obstacle in that direction).
    float sector[36];
    for (int s = 0; s < 36; ++s) sector[s] = MAX_RANGE;
    for (int a = 0; a < 360; ++a) {
        float r = ranges[a];
        if (r <= 0.0f || r > MAX_RANGE) continue;
        int s = (a / 10) % 36;
        if (r < sector[s]) sector[s] = r;
    }

    auto sectorMin = [&](int from, int to) -> float {
        float m = MAX_RANGE;
        for (int s = from % 36; s != (to % 36); s = (s + 1) % 36)
            if (sector[s] < m) m = sector[s];
        return m;
    };

    float minFrontNarrow = MAX_RANGE;
    { float m1 = sectorMin(0,4), m2 = sectorMin(33,36); minFrontNarrow = (m1<m2)?m1:m2; }
    float minFront = MAX_RANGE;
    { float m1 = sectorMin(0,5), m2 = sectorMin(31,36); minFront = (m1<m2)?m1:m2; }

    float minRightSide = sectorMin(5,  13);
    float minLeftSide  = sectorMin(23, 31);
    float minRight     = sectorMin(5,  18);
    float minLeft      = sectorMin(19, 35);
    float spaceRight   = sectorMin(1,  18);
    float spaceLeft    = sectorMin(19, 36);
    float minRear      = sectorMin(15, 21);

    bool inCorridor = (minFront >= BRAKE_MM) &&
                      (minRightSide < SAFE_MM) && (minLeftSide < SAFE_MM);

    ScanAnalysis res{};
    res.spaceLeft  = spaceLeft;
    res.spaceRight = spaceRight;
    res.minFront   = minFront;
    res.minLeft    = minLeft;
    res.minRight   = minRight;
    res.minRear    = minRear;
    res.avoidDir   = g_prevAvoid;

    if (minFront >= SAFE_MM) { res.state = 0; g_prevAvoid = 0.0f; return res; }
    if (inCorridor)          { res.state = 0; g_prevAvoid = 0.0f; return res; }

    // Choose avoidance direction: most free space wins, with hysteresis
    // so it does not flip back and forth between left and right.
    bool rightSafe = (minRightSide > CHASSIS_MM);
    bool leftSafe  = (minLeftSide  > CHASSIS_MM);
    float scoreRight = rightSafe ? spaceRight : 0.0f;
    float scoreLeft  = leftSafe  ? spaceLeft  : 0.0f;
    if (g_prevAvoid > 0.0f) scoreRight += HYSTERESIS;
    if (g_prevAvoid < 0.0f) scoreLeft  += HYSTERESIS;

    float newAvoid = (!rightSafe && !leftSafe)
        ? ((spaceRight >= spaceLeft) ? +1.0f : -1.0f)
        : ((scoreRight >= scoreLeft) ? +1.0f : -1.0f);

    res.avoidDir = newAvoid;
    g_prevAvoid  = newAvoid;

    if      (minFrontNarrow < CRITICAL_MM) res.state = 3;
    else if (minFront       < BRAKE_MM)    res.state = 2;
    else                                    res.state = 1;
    return res;
}


// ─────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────

Navigator::Navigator()
    : path()
    , currentTarget(0.0f, 0.0f, 0.0f)
    , isUpdated(false)
    , hasPath(false)
    , stableLin(0.0f)
    , stableAng(0.0f)
    , hasStableCmd(false)
    , cmdTicks(0)
    , blockCounter(0)
    , recoveryTicks(0)
    , blocked(false)
    , wallState(WallState::OPEN_SPACE)
    , wfFilteredError(0.0f)
    , outerCornerTicks(0)
{}


// ─────────────────────────────────────────────────────────────────
// SetPath / Update — path management
// ─────────────────────────────────────────────────────────────────

void Navigator::SetPath(const Path& newPath) {
    path = newPath;
    path.Reset();
    hasPath      = !path.IsEmpty();
    hasStableCmd = false;
    cmdTicks     = 0;
    blockCounter = 0;
    // recoveryTicks is NOT reset: if a recovery action (e.g. reversing) is
    // still running when replanning happens, let it finish so the robot is
    // physically clear of the obstacle before following the new path.
    blocked      = false;
    if (hasPath) currentTarget = path.GetCurrentWaypoint();
    isUpdated = true;
}

void Navigator::Update(Position current) {
    if (!hasPath || path.IsEmpty()) return;
    // Skip reached waypoints; on each new waypoint we recompute the
    // command immediately (reset the stable counter).
    while (ReachedPoint(current) && !path.IsEmpty()) {
        path.Advance();
        if (!path.IsEmpty()) {
            currentTarget = path.GetCurrentWaypoint();
            hasStableCmd  = false;
            cmdTicks      = 0;
        } else {
            hasPath = false;
        }
    }
    isUpdated = true;
}


// ─────────────────────────────────────────────────────────────────
// computeLinearSpeed — scale linear speed based on heading error
//
// The robot drives at full LINEAR_SPEED_MM_S only when it is already
// pointed at the target (small angle error). As the error grows the
// speed is reduced so the robot slows into turns instead of sliding
// through them.
//
// Mapping:
//   |err| ≤ ANGLE_DEADBAND_DEG          → full speed (factor = 1.0)
//   |err| = SLOW_TURN_THRESHOLD         → SLOW_TURN_FACTOR × full speed
//   |err| ≥ SLOW_TURN_THRESHOLD         → speed handled by the turn branch,
//                                          this function is not called
// ─────────────────────────────────────────────────────────────────

float Navigator::computeLinearSpeed(float absErr) const
{
    // The usable range is [ANGLE_DEADBAND_DEG, SLOW_TURN_THRESHOLD].
    // Outside that range the caller already selects a different motion mode.
    float range = SLOW_TURN_THRESHOLD - ANGLE_DEADBAND_DEG;
    if (range < 1.0f) range = 1.0f; // guard against degenerate config

    // Linear interpolation: 1.0 at ANGLE_DEADBAND, SLOW_TURN_FACTOR at SLOW_TURN_THRESHOLD.
    float t = (absErr - ANGLE_DEADBAND_DEG) / range;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    float factor = 1.0f - t * (1.0f - SLOW_TURN_FACTOR);

    // Hard floor: never go below SLOW_TURN_FACTOR so the robot keeps moving.
    if (factor < SLOW_TURN_FACTOR) factor = SLOW_TURN_FACTOR;

    return LINEAR_SPEED_MM_S * factor;
}


// ─────────────────────────────────────────────────────────────────
// GetNextCommand — core of the waypoint follower + recovery
//
// Logic in order:
//   1. Finish any running recovery action (reverse or turn).
//   2. Obstacle in front → raise block counter; if too often → recover.
//   3. Waypoint reached → stop.
//   4. Repeat the stable command while still valid (avoids wobbling).
//   5. Otherwise compute a new command from the heading error.
// ─────────────────────────────────────────────────────────────────

DriveCommand Navigator::GetNextCommand(Position current, float minFront, float minRear) {
    if (!hasPath || path.IsEmpty())
        return DriveCommand(0.0f, 0.0f);

    // -- 1. Running recovery action -------------------------------------------
    if (recoveryTicks > 0) {
        --recoveryTicks;
        return DriveCommand(stableLin, stableAng);
    }

    // -- 2. Obstacle interrupt + block detection -------------------------------
    if (minFront < OBSTACLE_INTERRUPT_MM) {
        ++blockCounter;
        hasStableCmd = false;
        cmdTicks     = 0;

        if (blockCounter >= RECOVERY_TRIGGER) {
            if (minRear > REVERSE_SAFE_MM) {
                stableLin     = REVERSE_SPEED;
                stableAng     = 0.0f;
                recoveryTicks = RECOVERY_TICKS;
                blocked       = true;
                printf("[NAV] RECOVERY reverse (front=%.0f rear=%.0f)\n", minFront, minRear);
            } else {
                float angleErr = CalculateAngleError(current, currentTarget);
                stableLin     = 0.0f;
                stableAng     = (angleErr > 0.0f) ? -MAX_ANGULAR_DEG_S : MAX_ANGULAR_DEG_S;
                recoveryTicks = RECOVERY_TICKS;
                blocked       = true;
                printf("[NAV] RECOVERY turn (front=%.0f rear=%.0f)\n", minFront, minRear);
            }
            blockCounter = 0;
            return DriveCommand(stableLin, stableAng);
        }

        return DriveCommand(0.0f, 0.0f);
    }

    // No obstacle → wind the block counter back down
    if (blockCounter > 0) --blockCounter;

    float dist     = CalculateDistance(current, currentTarget);
    float angleErr = CalculateAngleError(current, currentTarget);

    // -- 3. Waypoint reached --------------------------------------------------
    if (dist < REACHED_THRESHOLD_MM) {
        hasStableCmd = false;
        cmdTicks     = 0;
        return DriveCommand(0.0f, 0.0f);
    }

    // -- 4. Repeat the stable command -----------------------------------------
    bool grooteDraai = (std::fabs(angleErr) > SLOW_TURN_THRESHOLD);
    if (hasStableCmd && (cmdTicks < CMD_STABLE_TICKS || grooteDraai)) {
        ++cmdTicks;
        return DriveCommand(stableLin, stableAng);
    }

    // -- 5. Compute a new command ---------------------------------------------
    float absErr = std::fabs(angleErr);
    float newLin, newAng;

    if (absErr > SLOW_TURN_THRESHOLD) {
        // Large heading error → turn in place.
        // Lock the direction once to prevent flipping around ±180°.
        if (!hasStableCmd) {
            // Negative angleErr means target is to the right → turn right (+ang).
            newAng = (angleErr < 0.0f) ? MAX_ANGULAR_DEG_S : -MAX_ANGULAR_DEG_S;
        } else {
            newAng = stableAng; // keep already-chosen direction
        }
        newLin = 0.0f;

    } else if (absErr > ANGLE_DEADBAND_DEG) {
        // Medium heading error → move while gently steering.
        // Speed is scaled down so the robot slows before tight turns.
        newLin = computeLinearSpeed(absErr);
        newAng = ANGULAR_GAIN * absErr;
        if (newAng > MAX_ANGULAR_DEG_S) newAng = MAX_ANGULAR_DEG_S;
        if (angleErr > 0.0f) newAng = -newAng;

    } else {
        // Small heading error → drive straight at speed scaled to remaining error.
        // This avoids an abrupt jump from slow-steer speed to full speed the
        // moment the error drops below ANGLE_DEADBAND_DEG.
        newLin = computeLinearSpeed(absErr);
        newAng = 0.0f;
    }

    stableLin    = newLin;
    stableAng    = newAng;
    hasStableCmd = true;
    cmdTicks     = 0;

    printf("[NAV] cmd lin=%.0f ang=%.1f (dist=%.0fmm err=%.1fdeg)\n",
           newLin, newAng, dist, angleErr);

    return DriveCommand(stableLin, stableAng);
}


// ─────────────────────────────────────────────────────────────────
// Simple accessors
// ─────────────────────────────────────────────────────────────────

bool     Navigator::IsFinished()       const { return !hasPath || path.IsEmpty(); }
bool     Navigator::IsUpdated()        const { return isUpdated; }
Position Navigator::GetCurrentTarget() const { return currentTarget; }
Path     Navigator::GetPath()          const { return path; }

bool Navigator::ReachedPoint(Position current) const {
    return !path.IsEmpty() &&
           CalculateDistance(current, currentTarget) < REACHED_THRESHOLD_MM;
}

float Navigator::CalculateDistance(Position a, Position b) const {
    float dx = b.GetX() - a.GetX(), dy = b.GetY() - a.GetY();
    return std::sqrt(dx*dx + dy*dy);
}

float Navigator::CalculateAngleError(Position current, Position target) const {
    float dx = target.GetX() - current.GetX();
    float dy = target.GetY() - current.GetY();
    // atan2 returns a CCW angle, but theta comes from the IMU which is
    // CW-positive (compass convention).  The LIDAR CW-angle convention also
    // causes the Y-axis in the map to be mirrored relative to standard math.
    // Both effects flip the sign of the angular error, so we negate the
    // result here.  The rest of GetNextCommand is written with the
    // convention "negative error = target is to the right = turn right",
    // which remains correct after this negation.
    float desired = std::atan2(dy, dx) * (180.0f / static_cast<float>(M_PI));
    return NormalizeDeg(-(desired - current.GetTheta()));
}

float Navigator::NormalizeDeg(float deg) const {
    return NormDeg(deg); }


// ─────────────────────────────────────────────────────────────────
// Wall follower — sector helpers
// ─────────────────────────────────────────────────────────────────

float Navigator::WfSectorMin(const float ranges[360], int from, int to) const {
    static constexpr float MAX_R = 8000.0f;
    float m = MAX_R;
    for (int a = from; a < to; ++a) {
        int idx = ((a % 360) + 360) % 360;
        float r = ranges[idx];
        if (r > 0.0f && r < m) m = r;
    }
    return m;
}

float Navigator::WfSectorAvg(const float ranges[360], int from, int to) const {
    static constexpr float MAX_R = 8000.0f;
    float sum = 0.0f; int cnt = 0;
    for (int a = from; a < to; ++a) {
        int idx = ((a % 360) + 360) % 360;
        float r = ranges[idx];
        if (r > 0.0f && r < MAX_R) { sum += r; ++cnt; }
    }
    return cnt > 0 ? sum / static_cast<float>(cnt) : MAX_R;
}

void Navigator::ResetWallFollower() {
    wallState        = WallState::OPEN_SPACE;
    wfFilteredError  = 0.0f;
    outerCornerTicks = 0;
}


// ─────────────────────────────────────────────────────────────────
// ComputeWallCommand — right-hand wall follower
// ─────────────────────────────────────────────────────────────────

WallResult Navigator::ComputeWallCommand(const float ranges[360]) {
    float minFrontNarrow = std::min(WfSectorMin(ranges, 350, 360), WfSectorMin(ranges, 0, 10));
    float minFront       = std::min(WfSectorMin(ranges, 330, 360), WfSectorMin(ranges, 0, 30));
    float wallRight      = WfSectorAvg(ranges,  75, 105);
    float frontRight     = WfSectorMin(ranges,  30,  75);

    bool wallInFront      = (minFrontNarrow < WF_FRONT_CRITICAL_MM);
    bool wallFrontLikely  = (minFront       < WF_FRONT_BRAKE_MM);
    bool rightWallPresent = (wallRight      < WF_WALL_PRESENT_MM);
    bool frontRightClear  = (frontRight     > WF_WALL_PRESENT_MM * 1.3f);

    WallResult res{DriveCommand(0.0f, 0.0f), WallState::OPEN_SPACE, 0.0f};

    // CASE 1: wall straight ahead → inner corner, turn left
    if (wallInFront) {
        wallState = WallState::INNER_CORNER; wfFilteredError = 0.0f; outerCornerTicks = 0;
        res.cmd   = DriveCommand(0.0f, -WF_INNER_CORNER_TURN);
        res.state = WallState::INNER_CORNER; res.errorMm = 0.0f;
        printf("[WALL] INNER_CORNER front=%.0fmm -> turn left\n", minFrontNarrow);
        return res;
    }

    // CASE 2: right wall gone + front-right open → outer corner
    // (only valid if we were actually following a wall)
    if (!rightWallPresent && frontRightClear && wallState == WallState::FOLLOW_RIGHT) {
        wallState = WallState::OUTER_CORNER; outerCornerTicks = 0; wfFilteredError = 0.0f;
        printf("[WALL] OUTER_CORNER wallR=%.0fmm frontR=%.0fmm -> turn right\n", wallRight, frontRight);
    }

    if (wallState == WallState::OUTER_CORNER) {
        if (rightWallPresent && wallRight < WF_TARGET_DIST_MM * 1.5f) {
            wallState = WallState::FOLLOW_RIGHT; outerCornerTicks = 0;
            printf("[WALL] wall regained at %.0fmm -> FOLLOW_RIGHT\n", wallRight);
        } else if (++outerCornerTicks > WF_OUTER_CORNER_MAX_TICKS) {
            wallState = WallState::OPEN_SPACE; outerCornerTicks = 0;
            printf("[WALL] OUTER_CORNER timeout -> OPEN_SPACE\n");
        } else {
            res.cmd   = DriveCommand(WF_LIN_SLOW, WF_OUTER_CORNER_TURN);
            res.state = WallState::OUTER_CORNER; res.errorMm = 0.0f;
            return res;
        }
    }

    // CASE 3: no wall on the right → open space (MainPi5 switches to frontier)
    if (!rightWallPresent) {
        wallState   = WallState::OPEN_SPACE;
        float lin   = wallFrontLikely ? WF_LIN_SLOW : WF_LIN_NORMAL;
        printf("[WALL] OPEN_SPACE front=%.0fmm right=%.0fmm\n", minFront, wallRight);
        res.cmd   = DriveCommand(lin, 0.0f);
        res.state = WallState::OPEN_SPACE; res.errorMm = 0.0f;
        return res;
    }

    // CASE 4: normal right-wall following with a P controller on the distance error
    wallState = WallState::FOLLOW_RIGHT;
    float rawError  = wallRight - WF_TARGET_DIST_MM;
    wfFilteredError = WF_EMA * rawError + (1.0f - WF_EMA) * wfFilteredError;

    float corrDegS = WF_KP * wfFilteredError;
    if (corrDegS >  WF_MAX_CORR) corrDegS =  WF_MAX_CORR;
    if (corrDegS < -WF_MAX_CORR) corrDegS = -WF_MAX_CORR;
    if (std::fabs(corrDegS) < WF_MIN_CORR) corrDegS = 0.0f; // dead zone against wobble

    float lin = wallFrontLikely ? WF_LIN_SLOW : WF_LIN_NORMAL;
    printf("[WALL] FOLLOW_RIGHT wallR=%.0fmm err=%.0fmm corr=%.1f lin=%.0f\n",
           wallRight, wfFilteredError, corrDegS, lin);

    res.cmd   = DriveCommand(lin, corrDegS);
    res.state = WallState::FOLLOW_RIGHT; res.errorMm = wfFilteredError;
    return res;
}