#ifndef SENSORHUB_H
#define SENSORHUB_H

#include "IMU.h"
#include "Encoder.h"
#include "DateTime.h"
#include "LIDAR.h"

class SensorHub {

private:
    IMU imu;
    Encoder encoderLeft;
    Encoder encoderRight;
    LIDAR lidar;

    DateTime lastUpdate;
    bool sensorsUpdated;

public:
    SensorHub(int encLeft, int encLeftRes,
              int encRight, int encRightRes,
              int imuSDAPin, int imuSCLPin,
              uart_inst_t* lidarUart, int lidarBaud = 460800);

    bool UpdateSensors();

    float GetSpeedLeft() const;
    float GetSpeedRight() const;
    float GetCurrentYaw() const;
    DateTime GetLastUpdate() const;

    bool IsLidarObjectInRange(int minAngle, int maxAngle, int threshold) const;
};

#endif