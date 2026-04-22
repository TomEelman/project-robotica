#include "pico/stdlib.h"
#include "Robot.h"
#include "Drive.h"
#include <cstdio>
// ----------------------------------------
// Test states
// ----------------------------------------
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
    FORWARD_TURN_LEFT,
    PAUSE_6,
    BACKWARD_TURN_RIGHT,
    PAUSE_7,
    BACKWARD_TURN_LEFT,
    PAUSE_8,
    TEST_DONE
};

static TestState state     = TURN_RIGHT_90;
static uint32_t  tickCount = 0;

static constexpr uint32_t MS(uint32_t ms) { return ms / 10; }

static constexpr uint32_t TURN_90_TICKS = MS(3000);
static constexpr uint32_t PAUSE_TICKS   = MS(500);
static constexpr uint32_t DRIVE_TICKS   = MS(2000);
static constexpr uint32_t CURVE_TICKS   = MS(2000);

DriveCommand GetCommand(TestState s)
{
    switch (s)
    {
        case TURN_RIGHT_90:       return DriveCommand(   0.0f,  35.0f);
        case TURN_LEFT_90:        return DriveCommand(   0.0f, -35.0f);
        case DRIVE_FORWARD:       return DriveCommand( 378.0f,   0.0f);
        case DRIVE_BACKWARD:      return DriveCommand(-378.0f,   0.0f);
        case FORWARD_TURN_RIGHT:  return DriveCommand( 378.0f,  18.0f);
        case FORWARD_TURN_LEFT:   return DriveCommand( 378.0f, -18.0f);
        case BACKWARD_TURN_RIGHT: return DriveCommand(-378.0f,  18.0f);
        case BACKWARD_TURN_LEFT:  return DriveCommand(-378.0f, -18.0f);
        default:                  return DriveCommand(   0.0f,   0.0f);
    }
}

uint32_t DurationTicks(TestState s)
{
    switch (s)
    {
        case TURN_RIGHT_90:
        case TURN_LEFT_90:        return TURN_90_TICKS;
        case DRIVE_FORWARD:
        case DRIVE_BACKWARD:      return DRIVE_TICKS;
        case FORWARD_TURN_RIGHT:
        case FORWARD_TURN_LEFT:
        case BACKWARD_TURN_RIGHT:
        case BACKWARD_TURN_LEFT:  return CURVE_TICKS;
        default:                  return PAUSE_TICKS;
    }
}

// ----------------------------------------
// Main
// ----------------------------------------
int main()
{
    stdio_init_all();
    sleep_ms(2000);

    Robot robot;

    while (true)
    {
        // 1. Sensoren bijwerken (altijd)
        robot.UpdateSensors();

        // 2. Test state machine
        if (state == TEST_DONE)
        {
            robot.Execute(DriveCommand(0.0f, 0.0f));
        }
        else
        {
            DriveCommand cmd = GetCommand(state);
            robot.Execute(cmd);

            tickCount++;
            if (tickCount >= DurationTicks(state))
            {
                printf("State %d klaar na %lu ticks\n", state, tickCount);
                state     = static_cast<TestState>(static_cast<int>(state) + 1);
                tickCount = 0;
            }
        }

        sleep_ms(10);
    }
}