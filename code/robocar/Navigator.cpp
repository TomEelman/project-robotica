#include "Navigator.h"
#include "Localisation.h"
#include "Mapper.h"
#include "PathPlanner.h" 
#include "MainPi5.h"            
#include <cmath>
#include <cstdio>

float NormDeg(float deg) {
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static float g_prevAvoid = 0.0f;

ScanAnalysis AnalyzeScan(const float ranges[360]) {
    static constexpr float CRITICAL_MM = 350.0f;
    static constexpr float BRAKE_MM    = 500.0f;
    static constexpr float SAFE_MM     = 700.0f;
    static constexpr float CHASSIS_MM  = 280.0f;
    static constexpr float MAX_RANGE   = 8000.0f;
    static constexpr float HYSTERESIS  = 200.0f;

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

    bool inCorridor = (minFront >= BRAKE_MM) && (minRightSide < SAFE_MM) && (minLeftSide < SAFE_MM);

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
    , wallState(NavMode_WALLFOLLOWING::OPEN_SPACE)
    , wfFilteredError(0.0f)
    , outerCornerTicks(0)
{}

void Navigator::SetPath(const Path& newPath) {
    path = newPath;
    path.Reset();
    hasPath      = !path.IsEmpty();
    hasStableCmd = false;
    cmdTicks     = 0;
    blockCounter = 0;

    blocked      = false;
    if (hasPath) currentTarget = path.GetCurrentWaypoint();
    isUpdated = true;
}

void Navigator::Update(Position current) {
    if (!hasPath || path.IsEmpty()) return;

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

float Navigator::computeLinearSpeed(float absErr) const {
    float range = SLOW_TURN_THRESHOLD - ANGLE_DEADBAND_DEG;
    if (range < 1.0f) range = 1.0f;

    float t = (absErr - ANGLE_DEADBAND_DEG) / range;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    float factor = 1.0f - t * (1.0f - SLOW_TURN_FACTOR);

    if (factor < SLOW_TURN_FACTOR) factor = SLOW_TURN_FACTOR;

    return LINEAR_SPEED_MM_S * factor;
}

DriveCommand Navigator::GetNextCommand(Position current, float minFront, float minRear) {
    if (!hasPath || path.IsEmpty())
        return DriveCommand(0.0f, 0.0f);

    if (recoveryTicks > 0) {
        --recoveryTicks;
        return DriveCommand(stableLin, stableAng);
    }

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
            } else {
                float angleErr = CalculateAngleError(current, currentTarget);
                stableLin     = 0.0f;
                stableAng     = (angleErr > 0.0f) ? -MAX_ANGULAR_DEG_S : MAX_ANGULAR_DEG_S;
                recoveryTicks = RECOVERY_TICKS;
                blocked       = true;
            }
            blockCounter = 0;
            return DriveCommand(stableLin, stableAng);
        }

        return DriveCommand(0.0f, 0.0f);
    }

    if (blockCounter > 0) --blockCounter;

    float dist     = CalculateDistance(current, currentTarget);
    float angleErr = CalculateAngleError(current, currentTarget);

    if (dist < REACHED_THRESHOLD_MM) {
        hasStableCmd = false;
        cmdTicks     = 0;
        return DriveCommand(0.0f, 0.0f);
    }

    bool bigTurn = (std::fabs(angleErr) > SLOW_TURN_THRESHOLD);
    if (hasStableCmd && (cmdTicks < CMD_STABLE_TICKS || bigTurn)) {
        ++cmdTicks;
        return DriveCommand(stableLin, stableAng);
    }

    float absErr = std::fabs(angleErr);
    float newLin, newAng;

    if (absErr > SLOW_TURN_THRESHOLD) {
        if (!hasStableCmd) {
            newAng = (angleErr < 0.0f) ? MAX_ANGULAR_DEG_S : -MAX_ANGULAR_DEG_S;
        } else {
            newAng = stableAng;
        }
        newLin = 0.0f;

    } else if (absErr > ANGLE_DEADBAND_DEG) {
        newLin = computeLinearSpeed(absErr);
        newAng = ANGULAR_GAIN * absErr;
        if (newAng > MAX_ANGULAR_DEG_S) newAng = MAX_ANGULAR_DEG_S;
        if (angleErr > 0.0f) newAng = -newAng;

    } else {
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

    float desired = std::atan2(dy, dx) * (180.0f / static_cast<float>(M_PI));
    return NormalizeDeg(-(desired - current.GetTheta()));
}

float Navigator::NormalizeDeg(float deg) const {
    return NormDeg(deg); }

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
    wallState        = NavMode_WALLFOLLOWING::OPEN_SPACE;
    wfFilteredError  = 0.0f;
    outerCornerTicks = 0;
}

WallResult Navigator::ComputeWallCommand(const float ranges[360]) {
    float minFrontNarrow = std::min(WfSectorMin(ranges, 350, 360), WfSectorMin(ranges, 0, 10));
    float minFront       = std::min(WfSectorMin(ranges, 330, 360), WfSectorMin(ranges, 0, 30));
    float wallRight      = WfSectorAvg(ranges,  75, 105);
    float frontRight     = WfSectorMin(ranges,  30,  75);

    bool wallInFront      = (minFrontNarrow < WF_FRONT_CRITICAL_MM);
    bool wallFrontLikely  = (minFront       < WF_FRONT_BRAKE_MM);
    bool rightWallPresent = (wallRight      < WF_WALL_PRESENT_MM);
    bool frontRightClear  = (frontRight     > WF_WALL_PRESENT_MM * 1.3f);

    WallResult res{DriveCommand(0.0f, 0.0f), NavMode_WALLFOLLOWING::OPEN_SPACE, 0.0f};

    if (wallInFront) {
        wallState = NavMode_WALLFOLLOWING::INNER_CORNER; wfFilteredError = 0.0f; outerCornerTicks = 0;
        res.cmd   = DriveCommand(0.0f, -WF_INNER_CORNER_TURN);
        res.state = NavMode_WALLFOLLOWING::INNER_CORNER; res.errorMm = 0.0f;
        return res;
    }

    if (!rightWallPresent && frontRightClear && wallState == NavMode_WALLFOLLOWING::FOLLOW_RIGHT) {
        wallState = NavMode_WALLFOLLOWING::OUTER_CORNER; outerCornerTicks = 0; wfFilteredError = 0.0f;
    }

    if (wallState == NavMode_WALLFOLLOWING::OUTER_CORNER) {
        if (rightWallPresent && wallRight < WF_TARGET_DIST_MM * 1.5f) {
            wallState = NavMode_WALLFOLLOWING::FOLLOW_RIGHT; outerCornerTicks = 0;
        } else if (++outerCornerTicks > WF_OUTER_CORNER_MAX_TICKS) {
            wallState = NavMode_WALLFOLLOWING::OPEN_SPACE; outerCornerTicks = 0;
        } else {
            res.cmd   = DriveCommand(WF_LIN_SLOW, WF_OUTER_CORNER_TURN);
            res.state = NavMode_WALLFOLLOWING::OUTER_CORNER; res.errorMm = 0.0f;
            return res;
        }
    }

    if (!rightWallPresent) {
        wallState   = NavMode_WALLFOLLOWING::OPEN_SPACE;
        float lin   = wallFrontLikely ? WF_LIN_SLOW : WF_LIN_NORMAL;
        res.cmd   = DriveCommand(lin, 0.0f);
        res.state = NavMode_WALLFOLLOWING::OPEN_SPACE; res.errorMm = 0.0f;
        return res;
    }

    wallState = NavMode_WALLFOLLOWING::FOLLOW_RIGHT;
    float rawError  = wallRight - WF_TARGET_DIST_MM;
    wfFilteredError = WF_EMA * rawError + (1.0f - WF_EMA) * wfFilteredError;

    float corrDegS = WF_KP * wfFilteredError;
    if (corrDegS >  WF_MAX_CORR) corrDegS =  WF_MAX_CORR;
    if (corrDegS < -WF_MAX_CORR) corrDegS = -WF_MAX_CORR;
    if (std::fabs(corrDegS) < WF_MIN_CORR) corrDegS = 0.0f;

    float lin = wallFrontLikely ? WF_LIN_SLOW : WF_LIN_NORMAL;

    res.cmd   = DriveCommand(lin, corrDegS);
    res.state = NavMode_WALLFOLLOWING::FOLLOW_RIGHT; res.errorMm = wfFilteredError;
    return res;
}

bool Navigator::HandleObstacleAvoidance(const ScanAnalysis& scan, Localisation& loc,
    NavMode_EVADING& evadingMode, float& goalAngle, float& turnDirection,
    int& clearDriveTicks, int& reverseTicks, int& stuckCounter, int& reverseEscalation,
    float& escapeX, float& escapeY, CommandKeepAlive& ka, float LIN_SPEED, int& scansSinceReplan)
{
    if (evadingMode == NavMode_EVADING::TURNING) {
        float headingError = NormDeg(goalAngle - loc.GetTheta());
        if (std::fabs(headingError) < 8.0f) {
            evadingMode = NavMode_EVADING::CLEARING;
            if (scan.minFront < 600.0f) {
                clearDriveTicks = 10 + (reverseEscalation * 8);
            } else {
                clearDriveTicks = 25 + (reverseEscalation * 8);
            }
            scansSinceReplan = 15;
            ka.SetCommand(LIN_SPEED, 0.0f);
        } else {
            if (scan.state >= 3) {
                turnDirection = -turnDirection;
                goalAngle = NormDeg(loc.GetTheta() - (turnDirection * 90.0f));
                ka.SetCommand(0.0f, turnDirection * 40.0f);
            } else {
                ka.SetCommand(0.0f, turnDirection * 40.0f);
            }
        }
    } else if (evadingMode == NavMode_EVADING::CLEARING) {
        if (scan.state >= 3) {
            stuckCounter++;
            if (stuckCounter >= 2) {
                reverseEscalation++;
                int scalingFactor = reverseEscalation * 8;
                if (scalingFactor > 40) { scalingFactor = 40; }
                reverseTicks = 18 + scalingFactor;
                evadingMode = NavMode_EVADING::REVERSING;
                ka.SetCommand(-LIN_SPEED * 0.6f, 0.0f);
            } else {
                turnDirection = -turnDirection;
                goalAngle = NormDeg(loc.GetTheta() - (turnDirection * 90.0f));
                evadingMode = NavMode_EVADING::TURNING;
                ka.SetCommand(0.0f, turnDirection * 40.0f);
            }
        } else {
            clearDriveTicks--;
            if (clearDriveTicks <= 0) {
                stuckCounter = 0;
                reverseEscalation = 0;
                evadingMode = NavMode_EVADING::NORMAL;
                scansSinceReplan = 15;
            } else {
                ka.SetCommand(LIN_SPEED, 0.0f);
            }
        }
    } else if (evadingMode == NavMode_EVADING::REVERSING) {
        bool reverseDone = false;
        if (scan.minRear < 300.0f) {
            reverseDone = true;
        } else {
            reverseTicks--;
            if (reverseTicks <= 0) {
                reverseDone = true;
            }
        }

        if (reverseDone) {
            float headingBias = 0.0f;
            goalAngle = NormDeg(loc.GetTheta() + headingBias);
            if (headingBias >= 0.0f) {
                turnDirection = 1.0f;
            } else {
                turnDirection = -1.0f;
            }
            evadingMode = NavMode_EVADING::TURNING;
            ka.SetCommand(0.0f, turnDirection * 40.0f);
            escapeX = loc.GetX();
            escapeY = loc.GetY();
        } else {
            ka.SetCommand(-LIN_SPEED * 0.6f, 0.0f);
        }
    }
    return true;
}

// Bepaalt naar welke kant de robot moet draaien wanneer er geen bereikbaar
// frontier-doel is. Kijkt naar de vrije ruimte links vs rechts in de LIDAR-scan
// en draait naar de ruimste kant. Een statische voorkeur zorgt dat hij niet
// heen-en-weer flipt tussen links en rechts wanneer beide kanten ongeveer gelijk zijn.
static float ChooseFreestTurn(const float* ranges)
{
    constexpr float MAX_RANGE  = 8000.0f;
    constexpr float TURN_DEG_S = 35.0f;   // draaisnelheid
    constexpr float HYSTERESIS = 300.0f;  // mm voorkeur voor de vorige kant

    static float prevDir = TURN_DEG_S;    // onthoud laatste draairichting

    // Gemiddelde vrije ruimte links (90..180 graden) en rechts (180..270 graden).
    // Index 0 = recht vooruit, oplopend met de klok mee.
    float sumRight = 0.0f; int nRight = 0;
    float sumLeft  = 0.0f; int nLeft  = 0;

    for (int a = 30; a <= 150; ++a) {           // rechterkant
        float r = ranges[a];
        if (r <= 0.0f || r > MAX_RANGE) r = MAX_RANGE;
        sumRight += r; ++nRight;
    }
    for (int a = 210; a <= 330; ++a) {          // linkerkant
        float r = ranges[a];
        if (r <= 0.0f || r > MAX_RANGE) r = MAX_RANGE;
        sumLeft += r; ++nLeft;
    }

    float avgRight = (nRight > 0) ? sumRight / static_cast<float>(nRight) : 0.0f;
    float avgLeft  = (nLeft  > 0) ? sumLeft  / static_cast<float>(nLeft)  : 0.0f;

    // Voorkeur voor de vorige richting om flip-flop te voorkomen.
    if (prevDir > 0.0f) avgRight += HYSTERESIS;
    else                avgLeft  += HYSTERESIS;

    float dir = (avgRight >= avgLeft) ? TURN_DEG_S : -TURN_DEG_S;
    prevDir = dir;
    return dir;
}

void Navigator::HandleFrontierMode(Mapper& mapper, Position pos, bool newScan,
    int& scansSinceReplan, NavMode_HEAD& navMode, bool& hasPath
    , CommandKeepAlive& ka, PathPlanner& planner, std::vector<BlacklistItem>& frontierBlacklist, 
    int REPLAN_SCANS, const float* lastRanges, float minFront)
{
    if (hasPath && !IsFinished()) {
        Update(pos);
        DriveCommand cmd = GetNextCommand(pos, minFront);
        ka.SetCommand(cmd.GetLinVelocity(), cmd.GetAngVelocity());

        if (IsBlocked()) {
           //  printf("[MAIN] Navigator blocked -> replan\n");
            Position blockedTarget = GetCurrentTarget();
            explorationPlanner.AddToBlackList(frontierBlacklist, blockedTarget.GetX(), blockedTarget.GetY());
            ResetBlock();
            hasPath = false;
            scansSinceReplan = REPLAN_SCANS;
        }

        if (IsBlocked()) {
            // printf("[MAIN] Navigator blocked -> wacht op stabiele lokalisatie\n");

            Position blockedTarget = GetCurrentTarget();
            explorationPlanner.AddToBlackList(frontierBlacklist, blockedTarget.GetX(), blockedTarget.GetY());
            ResetBlock();
            hasPath = false;
            scansSinceReplan = 0; 
        }

    } else {
        if (hasPath && IsFinished()) {
            hasPath = false;
            scansSinceReplan = REPLAN_SCANS;
        }
        if (newScan) {
            scansSinceReplan = 0;

            constexpr float OUTER_WALL_MIN = 9.0f;
            constexpr float INTERIOR_MIN   = 70.0f;
            float outerWallPct, interiorPct, relCoveragePct;
            mapper.GetRoomCoverage(outerWallPct, interiorPct, relCoveragePct);

            bool mapDone = (outerWallPct >= OUTER_WALL_MIN &&
                               interiorPct  >= INTERIOR_MIN);

            printf("[MAIN] FRONTIER rand=%.0f%% int=%.0f%% rel=%.0f%%%s\n",
                   outerWallPct, interiorPct, relCoveragePct,
                  mapDone ? " *** KAART KLAAR ***" : "");

            if (mapDone) {
                navMode = NavMode_HEAD::RETURN_HOME;
                hasPath = false;
                //printf("[MAIN] Kaart gemapped (rand=%.0f%% int=%.0f%%) -> TERUG_HOME\n",
                //       outerWallPct, interiorPct);
                Path path = planner.PlanPath(pos, Position(0.0f, 0.0f, 0.0f), mapper.GetMap());
                if (!path.IsEmpty()) { SetPath(path); mapper.SetWaypoints(path); hasPath = true; }
            } else {
                explorationPlanner.TickBlacklist(frontierBlacklist);
                Position goal = explorationPlanner.ChooseFrontierGoal(mapper, pos, lastRanges, frontierBlacklist);
                bool noGoal = (goal.GetX() == pos.GetX() && goal.GetY() == pos.GetY());

                if (noGoal) {
                    // Geen bereikbaar frontier-doel gevonden vanaf deze plek.
                    // Blacklist de huidige omgeving zodat ChooseFrontierGoal hier
                    // niet meteen opnieuw op vastloopt, en draai weg naar de kant
                    // met de meeste vrije ruimte (voorkomt heen-en-weer in een hoek).
                    explorationPlanner.AddToBlackList(frontierBlacklist, pos.GetX(), pos.GetY());

                    float turnDir = ChooseFreestTurn(lastRanges);
                    ka.SetCommand(0.0f, turnDir);
                } else {
                    Path path = planner.PlanPath(pos, goal, mapper.GetMap());
                    if (!path.IsEmpty()) {
                        SetPath(path);
                        mapper.SetWaypoints(path);
                        hasPath = true;
                    } else {
                        explorationPlanner.AddToBlackList(frontierBlacklist, goal.GetX(), goal.GetY());
                        ka.Stop();
                    }
                }
            }
        }
        // no new scan ka will repeat last command
    }
}

void Navigator::HandleReturnToHome(Mapper& mapper, Position pos, Position startPoint,
    bool& hasPath, CommandKeepAlive& ka, PathPlanner& planner, float HOME_THRESHOLD_MM, NavMode_HEAD& navMode)
{
    float dx = pos.GetX() - startPoint.GetX();
    float dy = pos.GetY() - startPoint.GetY();
    if (std::hypot(dx, dy) < HOME_THRESHOLD_MM) {
        ka.Stop();
        mapper.SaveDebugMap("kaart.pgm");
        printf("[MAIN] Kaart gemapped!\n");
        navMode = NavMode_HEAD::DONE;
        running = false;
        return;
    } else {
        if (!hasPath || IsFinished()) {
            hasPath = false;
            Path path = planner.PlanPath(pos, startPoint, mapper.GetMap());
            if (!path.IsEmpty()) {
                SetPath(path);
                mapper.SetWaypoints(path);
                hasPath = true;
            } else {
                float homeAngle = static_cast<float>(std::atan2(-dy, -dx)) * (180.0f / static_cast<float>(M_PI));
                ka.SetCommand(200.0f, NormDeg(homeAngle - pos.GetTheta()) * 0.5f);
            }
        }
        if (hasPath && !IsFinished()) {
            Update(pos);
            DriveCommand cmd = GetNextCommand(pos);
            ka.SetCommand(cmd.GetLinVelocity(), cmd.GetAngVelocity());
        }
    }
}