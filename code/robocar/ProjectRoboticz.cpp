#include "pico/stdlib.h"
#include "Robot.h"

int main() {
    stdio_init_all();
    sleep_ms(2000);  // wacht op USB Serial

    Robot robot;
    while (true) {
        

        robot.Update();
        sleep_ms(10);  // regelloop van 10 Hz
    }
}
