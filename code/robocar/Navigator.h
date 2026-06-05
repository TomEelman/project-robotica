#pragma once

#include "Path.h"
#include "Position.h"
#include "GridMap.h"
#include "DriveCommand.h"
#include <vector>

class Localisation;
class Mapper;
class PathPlanner;
class CommandKeepAlive;
struct BlacklistItem;

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

enum class NavMode_HEAD  { 
    FRONTIER, 
    RETURN_HOME, 
    DONE 
};
enum class NavMode_EVADING { 
    NORMAL, 
    TURNING, 
    CLEARING, 
    REVERSING 
};


enum class NavMode_WALLFOLLOWING {
    FOLLOW_RIGHT,
    OUTER_CORNER,
    INNER_CORNER,
    OPEN_SPACE,
};

struct WallResult {
    DriveCommand cmd;
    NavMode_WALLFOLLOWING    state;
    float        errorMm;  
};

class Navigator {
public:
    Navigator();

    void         SetPath(const Path& newPath);
    void         Update(Position current);

    DriveCommand GetNextCommand(Position current, float minFront = 8000.0f, float minRear  = 8000.0f);

    bool         IsFinished() const;
    bool         IsUpdated()  const;
    Position     GetCurrentTarget() const;
    Path         GetPath()          const;

    bool         IsBlocked() const { return blocked; }
    void         ResetBlock()      { blocked = false; }

    WallResult ComputeWallCommand(const float ranges[360]);
    void       ResetWallFollower();
    NavMode_WALLFOLLOWING  GetWallState() const { return wallState; }

    bool HandleObstacleAvoidance(const ScanAnalysis& scan, Localisation& loc,
        NavMode_EVADING& evadingMode, float& goalAngle, float& turnDirection,
        int& clearDriveTicks, int& reverseTicks, int& stuckCounter, int& reverseEscalation,
        float& escapeX, float& escapeY, CommandKeepAlive& ka, float LIN_SPEED, int& scansSinceReplan);

    void HandleFrontierMode(Mapper& mapper, Position pos, bool newScan,
        int& scansSinceReplan, int& failedCounter, NavMode_HEAD& navMode, bool& hasPath,
        CommandKeepAlive& ka, PathPlanner& planner, std::vector<BlacklistItem>& frontierBlacklist,
        int REPLAN_SCANS, int FAILED_THRESHOLD, const float* lastRanges, float minFront);

    void HandleReturnToHome(Mapper& mapper, Position pos, Position startPoint,
        bool& hasPath, CommandKeepAlive& ka, PathPlanner& planner, float HOME_THRESHOLD_MM, NavMode_HEAD& navMode);

private:
    static constexpr float REACHED_THRESHOLD_MM  = 200.0f;
    static constexpr float LINEAR_SPEED_MM_S     = 278.0f;
    static constexpr float ANGULAR_GAIN          = 2.5f;
    static constexpr float MAX_ANGULAR_DEG_S     = 45.0f;
    static constexpr float ANGLE_DEADBAND_DEG    = 12.0f;
    static constexpr float SLOW_TURN_THRESHOLD   = 60.0f;
    static constexpr float SLOW_TURN_FACTOR      = 0.6f;

    static constexpr int   CMD_STABLE_TICKS      = 3;      
    static constexpr float OBSTACLE_INTERRUPT_MM = 400.0f; 

    static constexpr int   RECOVERY_TRIGGER      = 5;      
    static constexpr float REVERSE_SAFE_MM       = 500.0f; 
    static constexpr float REVERSE_SPEED         = -278.0f;
    static constexpr int   RECOVERY_TICKS        = 15;    

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

    Path     path;
    Position currentTarget;
    bool     isUpdated;
    bool     hasPath;

    float stableLin      = 0.0f;
    float stableAng      = 0.0f;
    bool  hasStableCmd   = false;
    int   cmdTicks       = 0;

    int   blockCounter   = 0;      
    int   recoveryTicks  = 0;      
    bool  blocked        = false;  

    NavMode_WALLFOLLOWING wallState        = NavMode_WALLFOLLOWING::OPEN_SPACE;
    float     wfFilteredError  = 0.0f;
    int       outerCornerTicks = 0;

    bool  ReachedPoint(Position current) const;
    float CalculateDistance  (Position a, Position b) const;
    float CalculateAngleError(Position current, Position target) const;
    float NormalizeDeg(float deg) const;
    float WfSectorMin(const float ranges[360], int from, int to) const;
    float WfSectorAvg(const float ranges[360], int from, int to) const;
    float computeLinearSpeed(float absErr) const;
};