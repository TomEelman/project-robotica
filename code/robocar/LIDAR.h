#ifndef LIDAR_H
#define LIDAR_H

#include <string>
#include <cstdint>

#define SCAN_SIZE 360

#ifndef B460800
#define B460800 460800
#endif

class LIDAR {

private:
    std::string  port;
    int          fd;
    int          baudRate;
    int          maxRange;
    int          minRange;
    int          scanData[SCAN_SIZE];
    int          rotationSpeed;
    int          currentAngle;
    int          nextAngle;
    bool         updated;

    bool sendCommand(uint8_t cmd);
    bool readDescriptor();

public:
    LIDAR(const std::string& port, int baudRate = 460800);

    bool Connect();
    void Disconnect();

    bool Update();

    int  GetDistance(int angle) const;

    bool IsObjectInRange(int minAngle, int maxAngle, int threshold) const;

    void ApplyMotionCorrection(float currentYaw);
};

#endif