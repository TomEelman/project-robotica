#include "pico/stdlib.h"
#include "Robot.h"

int main() {
    stdio_init_all();
    sleep_ms(2000);  // wacht op USB Serial

    Robot robot;
    float lastTime = 0 ;
    while (true) {
    uint64_t now = time_us_64();
    float dt = (now - lastTime) / 1000000.0f;
    if (dt <= 0.000001f) dt = 0.000001f;

    lastTime = now;
        robot.Update(dt);
        sleep_ms(10);  // regelloop van 10 Hz
    }
}
