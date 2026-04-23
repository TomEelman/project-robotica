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
    FORWARD_TURN_LEFT,
    PAUSE_6,
    BACKWARD_TURN_RIGHT,
    PAUSE_7,
    BACKWARD_TURN_LEFT,
    PAUSE_8,
    TEST_DONE
};

static TestState state            = TURN_RIGHT_90;
static uint32_t  tickCount        = 0;
static float     accumulatedAngle = 0.0f;
static float     targetAngle      = 90.0f;
static float     startYaw         = 0.0f;
static bool      startYawSet      = false;  // zodat we startYaw maar 1x opslaan

static constexpr uint32_t MS(uint32_t ms) { return ms / 10; }
static constexpr uint32_t PAUSE_TICKS = MS(500);
static constexpr uint32_t DRIVE_TICKS = MS(2000);
static constexpr uint32_t CURVE_TICKS = MS(2000);


// Bovenaan toevoegen: ramp parameters
static constexpr float RAMP_UP_DEG   = 20.0f;   // hoek waarop max snelheid bereikt is
static constexpr float RAMP_DOWN_DEG = 20.0f;   // hoek voor het einde beginnen te remmen
static constexpr float TURN_SPEED_MAX =  35.0f;  // jouw huidige snelheid
static constexpr float TURN_SPEED_MIN =   8.0f;  // langzaamste snelheid (net genoeg om te bewegen)

// Nieuwe helper: berekent de draaisnelheid op basis van hoe ver gedraaid is
float GetRampedTurnSpeed(float accumulated, float target)
{
    float progress = fabsf(accumulated);      // hoeveel graden gedraaid (altijd positief)
    float remaining = fabsf(target) - progress; // hoeveel nog over

    float speed;

    if (progress < RAMP_UP_DEG)
    {
        // Ramp-up fase: lineair van min naar max
        float t = progress / RAMP_UP_DEG;
        speed = TURN_SPEED_MIN + t * (TURN_SPEED_MAX - TURN_SPEED_MIN);
    }
    else if (remaining < RAMP_DOWN_DEG)
    {
        // Ramp-down fase: lineair van max naar min
        float t = remaining / RAMP_DOWN_DEG;
        speed = TURN_SPEED_MIN + t * (TURN_SPEED_MAX - TURN_SPEED_MIN);
    }
    else
    {
        // Plateau: volle snelheid
        speed = TURN_SPEED_MAX;
    }

    // Richting behouden (positief = rechts, negatief = links)
    return (target > 0.0f) ? speed : -speed;
}

bool IsTurnState(TestState s)
{
    return (s == TURN_RIGHT_90 || s == TURN_LEFT_90);
}

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
        case DRIVE_FORWARD:
        case DRIVE_BACKWARD:      return DRIVE_TICKS;
        case FORWARD_TURN_RIGHT:
        case FORWARD_TURN_LEFT:
        case BACKWARD_TURN_RIGHT:
        case BACKWARD_TURN_LEFT:  return CURVE_TICKS;
        default:                  return PAUSE_TICKS;
    }
}

void StartState(TestState newState)
{
    state            = newState;
    tickCount        = 0;
    accumulatedAngle = 0.0f;
    startYawSet      = false;  // reset zodat startYaw opnieuw wordt opgeslagen

    switch (newState)
    {
        case TURN_RIGHT_90: targetAngle =  90.0f; break;
        case TURN_LEFT_90:  targetAngle = -90.0f; break;
        default:            targetAngle =   0.0f; break;
    }

    printf("==> State %d gestart | target: %.1f graden\n", newState, targetAngle);
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    Robot robot;

    while (true)
    {
        robot.UpdateSensors();

        if (state == TEST_DONE)
        {
            robot.Execute(DriveCommand(0.0f, 0.0f));
        }
        else
        {
            
            DriveCommand cmd = GetCommand(state);

            if (IsTurnState(state) && startYawSet)
            {
                float rampedOmega = GetRampedTurnSpeed(accumulatedAngle, targetAngle);
                cmd = DriveCommand(0.0f, rampedOmega);
            }

            robot.Execute(cmd);
            tickCount++;

            bool done = false;

            if (IsTurnState(state))
            {
                // startYaw eenmalig opslaan bij begin van deze state
                if (!startYawSet)
                {
                    startYaw    = robot.GetCurrentYaw();
                    startYawSet = true;
                }

                float currentYaw = robot.GetCurrentYaw();
                accumulatedAngle = currentYaw - startYaw;

                // Wrap -180 tot +180
                while (accumulatedAngle >  180.0f) accumulatedAngle -= 360.0f;
                while (accumulatedAngle < -180.0f) accumulatedAngle += 360.0f;

                printf("yaw: %.1f | gedraaid: %.1f / %.1f\n",
                       currentYaw, accumulatedAngle, targetAngle);

                // Positief target: wacht tot accumulatedAngle >= targetAngle
                // Negatief target: wacht tot accumulatedAngle <= targetAngle
                if (targetAngle > 0.0f)
                    done = (accumulatedAngle >= targetAngle);
                else
                    done = (accumulatedAngle <= targetAngle);
            }
            else
            {
                done = (tickCount >= DurationTicks(state));
            }

            if (done)
            {
                printf("State %d klaar | gedraaid: %.1f graden | ticks: %lu\n",
                       state, accumulatedAngle, tickCount);
                StartState(static_cast<TestState>(static_cast<int>(state) + 1));
            }
        }

        sleep_ms(10);
    }
}