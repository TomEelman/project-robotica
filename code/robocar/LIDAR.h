#ifndef LIDAR_H
#define LIDAR_H

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <string>

#define SCAN_SIZE 360

using namespace sl;

class LIDAR {
public:
    LiDAR(const std::string& port, int baudRate = 460800);
    ~LiDAR();

    bool Connect();
    void Disconnect();

    bool Update();

    int  GetDistance(int angle) const;
    bool IsObjectInRange(int minAngle, int maxAngle, int threshold) const;
    void ApplyMotionCorrection(float currentYaw);

private:
    std::string        port;
    int                baudRate;
    int                maxRange;
    int                minRange;
    int                scanData[SCAN_SIZE];

    IChannel*          channel;
    ILidarDriver*      drv;
};

#endif