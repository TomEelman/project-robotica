#ifndef LIDAR_H
#define LIDAR_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#pragma GCC diagnostic pop

#include <string>

#define SCAN_SIZE 360

using namespace sl;

class LIDAR {
public:
    struct ScanEntry {
        int   angle;
        float distance;
    };

    LIDAR(const std::string& port, int baudRate = 460800);
    ~LIDAR();

    bool Connect();
    void Disconnect();

    bool Update();

    ScanEntry GetDistance(int angle) const;
    bool      IsObjectInRange(int minAngle, int maxAngle, float threshold) const;
    void      ApplyMotionCorrection(float currentYaw);

private:
    std::string port;
    int         baudRate;
    float       maxRange;
    float       minRange;

    ScanEntry   scanData[SCAN_SIZE];

    IChannel*     channel;
    ILidarDriver* driver;
};

#endif