#include "pico/stdlib.h"
#include "Robot.h"
#include "Drive.h"
#include "PicoUARTHandler.h"
#include <cmath>
#include <cstdio>

static constexpr bool USE_UART_CONTROL = true;

// ════════════════════════════════════════════════════════════════
//  Modus 1: UART-gestuurd
// ════════════════════════════════════════════════════════════════
static void RunUartMode(Robot& robot, PicoUARTHandler& uart)
{
    DriveCommand cmd(0.0f, 0.0f);
    if (uart.ConsumePendingCmd(cmd))
        robot.Execute(cmd);
}

// ════════════════════════════════════════════════════════════════
//  Modus 2: Lokale testreeks (ongewijzigd)
// ════════════════════════════════════════════════════════════════
enum TestState {
    TURN_RIGHT_90, PAUSE_1, TURN_LEFT_90,  PAUSE_2,
    DRIVE_FORWARD, PAUSE_3, DRIVE_BACKWARD,PAUSE_4,
    FORWARD_TURN_RIGHT,  PAUSE_5, FORWARD_TURN_LEFT,   PAUSE_6,
    BACKWARD_TURN_RIGHT, PAUSE_7, BACKWARD_TURN_LEFT,  PAUSE_8,
    TEST_DONE
};
static TestState state = TURN_RIGHT_90;
static uint32_t  tickCount = 0;
static float     targetAngle = 90.0f, startYaw = 0.0f, accumulatedAngle = 0.0f;
static bool      startYawSet = false;

static constexpr uint32_t MS(uint32_t ms) { return ms / 10; }
static constexpr uint32_t PAUSE_TICKS = MS(500);
static constexpr uint32_t DRIVE_TICKS = MS(2000);
static constexpr uint32_t CURVE_TICKS = MS(2000);

static bool IsTurnState(TestState s) { return s==TURN_RIGHT_90||s==TURN_LEFT_90; }
static DriveCommand GetCommand(TestState s) {
    switch (s) {
        case TURN_RIGHT_90:       return DriveCommand(   0.0f,  50.0f);
        case TURN_LEFT_90:        return DriveCommand(   0.0f, -50.0f);
        case DRIVE_FORWARD:       return DriveCommand( 478.0f,   0.0f);
        case DRIVE_BACKWARD:      return DriveCommand(-478.0f,   0.0f);
        case FORWARD_TURN_RIGHT:  return DriveCommand( 478.0f,  8.0f);
        case FORWARD_TURN_LEFT:   return DriveCommand( 478.0f, -8.0f);
        case BACKWARD_TURN_RIGHT: return DriveCommand(-478.0f,  8.0f);
        case BACKWARD_TURN_LEFT:  return DriveCommand(-478.0f, -8.0f);
        default:                  return DriveCommand(   0.0f,  0.0f);
    }
}
static uint32_t DurationTicks(TestState s) {
    switch (s) {
        case DRIVE_FORWARD: case DRIVE_BACKWARD:           return DRIVE_TICKS;
        case FORWARD_TURN_RIGHT:  case FORWARD_TURN_LEFT:
        case BACKWARD_TURN_RIGHT: case BACKWARD_TURN_LEFT: return CURVE_TICKS;
        default: return PAUSE_TICKS;
    }
}
static void StartState(TestState s) {
    state=s; tickCount=0; accumulatedAngle=0.0f; startYawSet=false;
    switch(s){
        case TURN_RIGHT_90: targetAngle= 90.0f; break;
        case TURN_LEFT_90:  targetAngle=-90.0f; break;
        default:            targetAngle=  0.0f; break;
    }
}
static void RunTestMode(Robot& robot) {
    if (state==TEST_DONE) { robot.Execute(DriveCommand(0,0)); return; }
    robot.Execute(GetCommand(state));
    tickCount++;
    bool done = false;
    if (IsTurnState(state)) {
        if (!startYawSet) { startYaw=robot.GetCurrentYaw(); startYawSet=true; }
        accumulatedAngle = robot.GetCurrentYaw() - startYaw;
        while (accumulatedAngle >  180.0f) accumulatedAngle -= 360.0f;
        while (accumulatedAngle < -180.0f) accumulatedAngle += 360.0f;
        done = (targetAngle>0.0f) ? accumulatedAngle>=targetAngle
                                  : accumulatedAngle<=targetAngle;
    } else {
        done = tickCount >= DurationTicks(state);
    }
    if (done) {
        robot.Execute(DriveCommand(0,0));
        StartState(static_cast<TestState>(static_cast<int>(state)+1));
    }
}

// ════════════════════════════════════════════════════════════════
//  main
// ════════════════════════════════════════════════════════════════
int main()
{
    stdio_init_all();
    sleep_ms(2000);

    Robot            robot;
    PicoUARTHandler  uart(robot.getSenorHub());
    uart.Init();   // ← installeert de IRQ, vervangt SensorHub::InitUart()

    printf("Robot gestart | modus: %s\n",
           USE_UART_CONTROL ? "UART (RPi5-gestuurd)" : "LOKALE TESTREEKS");

    while (true)
    {
        robot.UpdateSensors();

        if (USE_UART_CONTROL)
            RunUartMode(robot, uart);
        else
            RunTestMode(robot);

        sleep_ms(10);
    }
}