#include "pico/stdlib.h"
#include "Robot.h"
#include "DriveCommand.h"
#include "PicoUARTHandler.h"
#include <cstdio>

// ════════════════════════════════════════════════════════════════
// MainPico  —  Pico main loop
//
// PicoUARTHandler.Tick() handles:
//   - Receiving CMD messages from the Pi5
//   - Automatically pushing DATA packets every 50 ms
//   - Command timeout watchdog (500 ms → emergency stop)
//
// MainPico only needs to:
//   1. Call Tick()
//   2. Execute commands when Tick() returns true
// ════════════════════════════════════════════════════════════════

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    Robot robot;
    PicoUARTHandler uart(robot.GetSensorHub());

    uart.Init();

    printf("[Pico] Started — waiting for CMD from Pi5\n");

    while (true) {
        absolute_time_t loopStart = get_absolute_time();


        DriveCommand cmd(0.0f, 0.0f);

        if (uart.Tick(cmd)) { // logic for uart handling with watchdog checks when there is an new cmd execute it
            robot.Execute(cmd); 
        }

        // Maintain loop frequency at approximately 100 Hz (10 ms cycle)
        int64_t elapsedUs = absolute_time_diff_us(loopStart, get_absolute_time());
        int64_t remainingUs = 10000 - elapsedUs;

        if (remainingUs > 0) {
            sleep_us(static_cast<uint32_t>(remainingUs));
        }
    }

    return 0;
}