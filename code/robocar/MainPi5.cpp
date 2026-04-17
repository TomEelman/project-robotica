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

        // Print all distances
        for (int angle = 0; angle < 360; angle++) {
            int dist = lidar.GetDistance(angle);
            if (dist > 0) {
                printf("Angle: %d Distance: %f mm\n", angle, (float)dist);
            }
        }

		ctrl_c_pressed = true;

        // if (lidar.IsObjectInRange(350, 360, 500) || lidar.IsObjectInRange(0, 10, 500)) {
        //     std::cout << "Object detected in front!\n";
        // }
    }

    std::cout << "\nStopping Disconnecting LiDAR...\n";
    lidar.Disconnect();
    return 0;
}