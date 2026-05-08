#include "pico/stdlib.h"
#include "Robot.h"
#include "Drive.h"
#include <cmath>
#include <cstdio>

enum TestState {
    TURN_RIGHT_90,
    PAUSE_1,
    TURN_LEFT_90,
    PAUSE_2,
    DRIVE_FORWARD,
    PAUSE_3,
    DRIVE_BACKWARD,
    PAUSE_4,
    FORWARD_TURN_RIGHT,
    PAUSE_5,
    BACKWARD_TURN_RIGHT,
    PAUSE_6,
    FORWARD_TURN_LEFT,
    PAUSE_7,
    BACKWARD_TURN_LEFT,
    PAUSE_8,
    TEST_DONE
};

// 100 Hz hoofd-loop → 1 tick = 10 ms
static constexpr uint32_t MS_TO_TICKS(uint32_t ms) { return ms / 10; }
static constexpr uint32_t PAUSE_TICKS = MS_TO_TICKS(500);
static constexpr uint32_t DRIVE_TICKS = MS_TO_TICKS(2000);
static constexpr uint32_t CURVE_TICKS = MS_TO_TICKS(2000);

static TestState state            = TURN_RIGHT_90;
static uint32_t  tickCount        = 0;
static float     accumulatedAngle = 0.0f;
static float     targetAngle      = 90.0f;
static float     startYaw         = 0.0f;
static bool      startYawSet      = false;


static bool IsTurnState(TestState s)
{
    return (s == TURN_RIGHT_90 || s == TURN_LEFT_90);
}

static DriveCommand GetCommand(TestState s)
{
    switch (s)
    {
        case TURN_RIGHT_90:       return DriveCommand(   0.0f,  50.0f);
        case TURN_LEFT_90:        return DriveCommand(   0.0f, -50.0f);
        case DRIVE_FORWARD:       return DriveCommand( 278.0f,   0.0f);
        case DRIVE_BACKWARD:      return DriveCommand(-278.0f,   0.0f);
        case FORWARD_TURN_RIGHT:  return DriveCommand( 278.0f,   8.0f);
        case FORWARD_TURN_LEFT:   return DriveCommand( 278.0f,  -8.0f);
        case BACKWARD_TURN_RIGHT: return DriveCommand(-278.0f,   8.0f);
        case BACKWARD_TURN_LEFT:  return DriveCommand(-278.0f,  -8.0f);
        default:                  return DriveCommand(   0.0f,   0.0f);
    }
}

static uint32_t DurationTicks(TestState s)
{
    switch (s)
    {
        case DRIVE_FORWARD:
        case DRIVE_BACKWARD:      return DRIVE_TICKS;
        case FORWARD_TURN_RIGHT:
        case FORWARD_TURN_LEFT:
        case BACKWARD_TURN_RIGHT:
        case BACKWARD_TURN_LEFT:  return CURVE_TICKS;
        default:                  return PAUSE_TICKS;
    }
}

static void StartState(TestState newState)
{
    state            = newState;
    tickCount        = 0;
    accumulatedAngle = 0.0f;
    startYawSet      = false;

    switch (newState)
    {
        case TURN_RIGHT_90: targetAngle =  90.0f; break;
        case TURN_LEFT_90:  targetAngle = -90.0f; break;
        default:            targetAngle =   0.0f; break;
    }
}
int main()
{
    stdio_init_all();
    sleep_ms(2000);

    Robot robot;

    constexpr float TARGET_ANGLE_DEG   = 90.0f;
    constexpr float MAX_ANGULAR_DEG_S  = 50.0f;
    constexpr float MIN_ANGULAR_DEG_S  = 8.0f;
    constexpr float BRAKE_ZONE_DEG     = 6.6f;
    constexpr float ANGLE_THRESHOLD_DEG = 0.5f;

    robot.UpdateSensors();

    float startYaw = robot.GetCurrentYaw();

    while (true)
    {
        robot.UpdateSensors();

        float currentYaw = robot.GetCurrentYaw();

        float accumulatedAngle = currentYaw - startYaw;

        while (accumulatedAngle > 180.0f)
        {
            accumulatedAngle -= 360.0f;
        }

        while (accumulatedAngle < -180.0f)
        {
            accumulatedAngle += 360.0f;
        }

        float error = TARGET_ANGLE_DEG - accumulatedAngle;
        float absError = std::fabs(error);

        // Target bereikt
        if (absError <= ANGLE_THRESHOLD_DEG)
        {
            robot.Execute(DriveCommand(0.0f, 0.0f));

            float measuredAngle = accumulatedAngle;
            float finalError = measuredAngle - TARGET_ANGLE_DEG;
            float errorPercent = (finalError / TARGET_ANGLE_DEG) * 100.0f;

            printf("\n=== TURN RESULT ===\n");
            printf("Target angle : %.2f deg\n", TARGET_ANGLE_DEG);
            printf("Measured     : %.2f deg\n", measuredAngle);
            printf("Error        : %.2f deg\n", finalError);
            printf("Error        : %.2f %%\n", errorPercent);

            break;
        }

        float angularDegS;

        // Ramp-down
        if (absError > BRAKE_ZONE_DEG)
        {
            angularDegS = MAX_ANGULAR_DEG_S;
        }
        else
        {
            float scale = absError / BRAKE_ZONE_DEG;

            angularDegS =
                MIN_ANGULAR_DEG_S +
                scale * (MAX_ANGULAR_DEG_S - MIN_ANGULAR_DEG_S);
        }

        // Richting behouden
        if (error < 0.0f)
        {
            angularDegS = -angularDegS;
        }

        robot.Execute(DriveCommand(0.0f, angularDegS));

        printf(
            "Angle: %7.2f deg | Error: %7.2f deg | Speed: %7.2f deg/s\n",
            accumulatedAngle,
            error,
            angularDegS
        );

        sleep_ms(10);
    }

    while (true)
    {
        sleep_ms(1000);
    }
}