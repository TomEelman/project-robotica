#include "LIDAR.h"
#include <iostream>
#include <csignal>
#include <unistd.h>

static volatile bool ctrl_c_pressed = false;

static void onSignal(int) {
    ctrl_c_pressed = true;
}

int main() {
    signal(SIGINT, onSignal);

    LIDAR lidar("/dev/ttyUSB0", 460800);

    if (!lidar.Connect()) {
        std::cerr << "Failed to connect to LIDAR\n";
        return 1;
    }

    std::cout << "LIDAR connected. Starting scan...\n";

    while (!ctrl_c_pressed) {
        if (!lidar.Update()) {
            std::cerr << "LIDAR update failed\n";
            break;
        }

        for (int angle = 0; angle < 1; angle++) {
            LIDAR::ScanEntry entry = lidar.GetDistance(angle);
            if (entry.distance > 0.0f) {
                printf("Angle: %d deg  Distance: %f mm\n", entry.angle, entry.distance);
            }
        }

        ctrl_c_pressed = true;
    }

    std::cout << "\nStopping, disconnecting LIDAR...\n";
    lidar.Disconnect();
    return 0;
}