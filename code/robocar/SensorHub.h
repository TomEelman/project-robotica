#ifndef SENSORHUB_H
#define SENSORHUB_H

#include "IMU.h"
#include "Encoder.h"
#include "DateTime.h"

class SensorHub {

private:
    IMU imu;
    Encoder encoderLeft;
    Encoder encoderRight;

    DateTime lastUpdate;
    bool sensorsUpdated;

public:
    SensorHub(int encLeft, int encLeftRes,
              int encRight, int encRightRes,
              int imuSDAPin, int imuSCLPin);

    bool UpdateSensors();

    float GetSpeedLeft() const;
    float GetDistanceLeft() const;
    float GetSpeedRight() const;
    float GetDistanceRight() const;
    float GetCurrentYaw() const;
    float GetAngVelocity() const;
    DateTime GetLastUpdate() const;
};

#endif