#pragma once

#include "Path.h"
#include "Position.h"
#include "GridMap.h"
#include "DriveCommand.h"

// ─────────────────────────────────────────────────────────────────
//  ScanAnalysis — result of a 360° LIDAR scan (obstacle avoidance)
//
//  state:       0=clear  1=safe  2=brake  3=critical
//  avoidDir:    +1=avoid right  -1=avoid left
// ─────────────────────────────────────────────────────────────────
struct ScanAnalysis {
    int   state;
    float avoidDir;
    float spaceLeft;
    float spaceRight;
    float minFront;
    float minLeft;
    float minRight;
    float minRear;
};

ScanAnalysis AnalyzeScan(const float ranges[360]);
float NormDeg(float deg);


// ─────────────────────────────────────────────────────────────────
//  WallState — wall-follower state
// ─────────────────────────────────────────────────────────────────
enum class WallState {
    FOLLOW_RIGHT,
    OUTER_CORNER,
    INNER_CORNER,
    OPEN_SPACE,
};

struct WallResult {
    DriveCommand cmd;
    WallState    state;
    float        errorMm;   // deviation from target distance (debug)
};


// ─────────────────────────────────────────────────────────────────
//  Navigator — waypoint follower + wall follower + recovery
// ─────────────────────────────────────────────────────────────────
class Navigator {
public:
    Navigator();

    // ── Waypoint follower ─────────────────────────────────────────
    void         SetPath(const Path& newPath);
    void         Update(Position current);

    // minFront / minRear in mm — used for obstacle interrupt and to
    // decide whether reversing is safe during recovery.
    DriveCommand GetNextCommand(Position current,
                                float minFront = 8000.0f,
                                float minRear  = 8000.0f);

    bool         IsFinished() const;
    bool         IsUpdated()  const;
    Position     GetCurrentTarget() const;
    Path         GetPath()          const;

    // True once the Navigator got stuck and abandoned the path. MainPi5
    // uses this as a signal to replan (replan-on-block).
    bool         IsBlocked() const { return blocked; }
    void         ResetBlock()      { blocked = false; }

    // Annuleert een lopende interne recovery (reverse/turn) volledig. Gebruikt
    // door MainPi5 wanneer die de recovery zelf overneemt (achteruit→wegdraaien),
    // zodat de Navigator-reverse niet alsnog opduikt na de herplanning.
    void         CancelRecovery()  { recoveryTicks = 0; blockCounter = 0; blocked = false; }

    // ── Wall follower ─────────────────────────────────────────────
    WallResult ComputeWallCommand(const float ranges[360]);
    void       ResetWallFollower();
    WallState  GetWallState() const { return wallState; }

private:
    // ── Waypoint follower constants ───────────────────────────────
    static constexpr float REACHED_THRESHOLD_MM  = 200.0f;
    static constexpr float LINEAR_SPEED_MM_S     = 278.0f;
    static constexpr float ANGULAR_GAIN          = 2.5f;
    static constexpr float MAX_ANGULAR_DEG_S     = 45.0f;
    static constexpr float ANGLE_DEADBAND_DEG    = 12.0f;
    static constexpr float SLOW_TURN_THRESHOLD   = 60.0f;
    static constexpr float SLOW_TURN_FACTOR      = 0.6f;

    // Stable command: only recompute after N ticks or on obstacle/event
    static constexpr int   CMD_STABLE_TICKS      = 3;      // herteken snel zodat slip wordt gecorrigeerd
    static constexpr float OBSTACLE_INTERRUPT_MM = 400.0f; // front stop threshold

    // ── Recovery constants ────────────────────────────────────────
    static constexpr int   RECOVERY_TRIGGER      = 5;      // # blocked ticks
    static constexpr float REVERSE_SAFE_MM       = 500.0f; // min clearance rear
    static constexpr float REVERSE_SPEED         = -278.0f;// straight reverse
    static constexpr int   RECOVERY_TICKS        = 15;     // duration of action

    // ── Wall follower constants ───────────────────────────────────
    static constexpr float WF_TARGET_DIST_MM     = 300.0f;
    static constexpr float WF_WALL_PRESENT_MM    = 600.0f;
    static constexpr float WF_FRONT_CRITICAL_MM  = 350.0f;
    static constexpr float WF_FRONT_BRAKE_MM     = 500.0f;
    static constexpr float WF_LIN_NORMAL         = 250.0f;
    static constexpr float WF_LIN_SLOW           = 150.0f;
    static constexpr float WF_KP                 = 0.08f;
    static constexpr float WF_MAX_CORR           = 30.0f;
    static constexpr float WF_MIN_CORR           = 3.0f;
    static constexpr float WF_EMA                = 0.25f;
    static constexpr float WF_OUTER_CORNER_TURN  = 25.0f;
    static constexpr float WF_INNER_CORNER_TURN  = 35.0f;
    static constexpr int   WF_OUTER_CORNER_MAX_TICKS = 20;

    // ── Waypoint follower state ───────────────────────────────────
    Path     path;
    Position currentTarget;
    bool     isUpdated;
    bool     hasPath;

    // Stable command state
    float stableLin      = 0.0f;
    float stableAng      = 0.0f;
    bool  hasStableCmd   = false;
    int   cmdTicks       = 0;

    // Recovery / block state
    int   blockCounter   = 0;      // consecutive blocked ticks
    int   recoveryTicks  = 0;      // remaining ticks of current recovery action
    bool  blocked        = false;  // signal to MainPi5 for replan

    // ── Wall follower state ───────────────────────────────────────
    WallState wallState        = WallState::OPEN_SPACE;
    float     wfFilteredError  = 0.0f;
    int       outerCornerTicks = 0;

    // ── Helpers ───────────────────────────────────────────────────
    bool  ReachedPoint(Position current) const;
    float CalculateDistance  (Position a, Position b) const;
    float CalculateAngleError(Position current, Position target) const;
    float NormalizeDeg(float deg) const;
    float WfSectorMin(const float ranges[360], int from, int to) const;
    float WfSectorAvg(const float ranges[360], int from, int to) const;
    float computeLinearSpeed(float absErr) const;
};