#ifndef SENSORHUB_H
#define SENSORHUB_H

#include "IMU.h"
#include "LIDAR.h"
#include "Encoder.h"
#include "DateTime.h"

class SensorHub {

private:
    LIDAR lidar;
    IMU imu;
    Encoder encoderLeft;
    Encoder encoderRight;
    DateTime lastUpdate;
    bool sensorsUpdated;
    float currentEncoderYaw;
 

public:
    SensorHub(int encLeft, int encLeftRes,
              int encRight, int encRightRes,
              int imuSDAPin, int imuSCLPin);
    bool UpdateSensors();
    float GetLidarScan() const;
    float GetCurrentYaw() const;
    float GetSpeedLeft() const;
    float GetSpeedRight() const;
    DateTime GetLastUpdate() const;
    float GetEncoderYaw();
    float GetAngVelocity() const;
};

#endif