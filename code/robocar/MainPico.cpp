#include "pico/stdlib.h"
#include "Robot.h"
#include "DriveCommand.h"
#include "PicoUARTHandler.h"
#include <cstdio>

// ════════════════════════════════════════════════════════════════
//  MainPico  —  Pico hoofdloop
//
//  PicoUARTHandler.Tick() doet alles:
//    - Ontvangt CMD van Pi5 en geeft die terug als DriveCommand
//    - Pusht elke 50ms automatisch DATA naar Pi5
//    - Watchdog: geen CMD binnen 500ms → stop
//
//  MainPico hoeft alleen:
//    1. Tick() aanroepen
//    2. Commando uitvoeren als Tick() true geeft
// ════════════════════════════════════════════════════════════════

int main()
{
    stdio_init_all();
    sleep_ms(2000);

    Robot robot;
    PicoUARTHandler uart(robot.GetSensorHub());

    uart.Init();

    printf("[Pico] Gestart — wacht op CMD van Pi5\n");

    while (true) {
        absolute_time_t loopStart = get_absolute_time();

        // Tick verwerkt UART, pusht DATA, checkt watchdog
        DriveCommand cmd(0.0f, 0.0f);
        if (uart.Tick(cmd))
            robot.Execute(cmd);

        // Loop op ~100 Hz houden (10 ms per cyclus)
        int64_t elapsed = absolute_time_diff_us(loopStart, get_absolute_time());
        int64_t rest    = 10000 - elapsed;
        if (rest > 0)
            sleep_us(static_cast<uint32_t>(rest));
    }

    return 0;
}