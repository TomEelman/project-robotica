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

    StartState(TURN_RIGHT_90);

    while (state != TEST_DONE)
    {
        robot.UpdateSensors();

        DriveCommand command = GetCommand(state);
        robot.Execute(command);

        if (IsTurnState(state))
        {
            float currentYaw = robot.GetCurrentYaw();

            if (!startYawSet)
            {
                startYaw = currentYaw;
                startYawSet = true;
            }

            accumulatedAngle = currentYaw - startYaw;

            while (accumulatedAngle > 180.0f)
            {
                accumulatedAngle -= 360.0f;
            }

            while (accumulatedAngle < -180.0f)
            {
                accumulatedAngle += 360.0f;
            }

            bool turnFinished = false;

            if (targetAngle > 0.0f)
            {
                turnFinished = (accumulatedAngle >= targetAngle);
            }
            else
            {
                turnFinished = (accumulatedAngle <= targetAngle);
            }

            if (turnFinished)
            {
                robot.Execute(DriveCommand(0.0f, 0.0f));

                printf("Turn finished\n");
                printf("Target angle: %.2f\n", targetAngle);
                printf("Measured angle: %.2f\n", accumulatedAngle);

                float error = accumulatedAngle - targetAngle;
                float errorPercent = (error / targetAngle) * 100.0f;

                printf("Error: %.2f deg\n", error);
                printf("Error: %.2f %%\n", errorPercent);

                switch (state)
                {
                    case TURN_RIGHT_90:
                        StartState(PAUSE_1);
                        break;

                    case TURN_LEFT_90:
                        StartState(PAUSE_2);
                        break;

                    default:
                        break;
                }
            }
        }
        else
        {
            tickCount++;

            if (tickCount >= DurationTicks(state))
            {
                switch (state)
                {
                    case PAUSE_1:
                        StartState(TURN_LEFT_90);
                        break;

                    case PAUSE_2:
                        StartState(DRIVE_FORWARD);
                        break;

                    case DRIVE_FORWARD:
                        StartState(PAUSE_3);
                        break;

                    case PAUSE_3:
                        StartState(DRIVE_BACKWARD);
                        break;

                    case DRIVE_BACKWARD:
                        StartState(PAUSE_4);
                        break;

                    case PAUSE_4:
                        StartState(FORWARD_TURN_RIGHT);
                        break;

                    case FORWARD_TURN_RIGHT:
                        StartState(PAUSE_5);
                        break;

                    case PAUSE_5:
                        StartState(BACKWARD_TURN_RIGHT);
                        break;

                    case BACKWARD_TURN_RIGHT:
                        StartState(PAUSE_6);
                        break;

                    case PAUSE_6:
                        StartState(FORWARD_TURN_LEFT);
                        break;

                    case FORWARD_TURN_LEFT:
                        StartState(PAUSE_7);
                        break;

                    case PAUSE_7:
                        StartState(BACKWARD_TURN_LEFT);
                        break;

                    case BACKWARD_TURN_LEFT:
                        StartState(PAUSE_8);
                        break;

                    case PAUSE_8:
                        StartState(TEST_DONE);
                        break;

                    default:
                        break;
                }
            }
        }

        sleep_ms(10);
    }

    robot.Execute(DriveCommand(0.0f, 0.0f));

    printf("Test sequence completed\n");

    while (true)
    {
        sleep_ms(1000);
    }
}

