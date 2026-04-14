#include "LIDAR.h"
#include <iostream>

int main() {
    LIDAR lidar("/dev/ttyUSB0");

    if (!lidar.Connect()) {
        std::cerr << "Failed to connect to LiDAR on /dev/ttyUSB0\n";
        return 1;
    }

    std::cout << "Connected. Scanning...\n";

    if (!lidar.Update()) {
        std::cerr << "Scan failed\n";
        lidar.Disconnect();
        return 1;
    }

    std::cout << "Scan complete:\n";
    std::cout << "Angle(deg)  Distance(mm)\n";
    std::cout << "------------------------\n";
    for (int angle = 0; angle < 360; angle++) {
        int dist = lidar.GetDistance(angle);
        if (dist > 0) {
            std::cout << angle << "°\t" << dist << " mm\n";
        }
    }

    lidar.Disconnect();
    return 0;
}
