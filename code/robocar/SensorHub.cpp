#include "SensorHub.h"
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

SensorHub::SensorHub(int encLeft, int encLeftRes,
                     int encRight, int encRightRes,
                     int imuSDAPin, int imuSCLPin)
    : encoderLeft(encLeft, encLeftRes),
      encoderRight(encRight, encRightRes),
      imu(imuSDAPin, imuSCLPin, 0x28)   
{
    sensorsUpdated = false;
}



bool SensorHub::UpdateSensors() {

    bool imuOk = imu.Update();
    bool leftOk = encoderLeft.Update();
    bool rightOk = encoderRight.Update();

    sensorsUpdated = imuOk && leftOk && rightOk;

    // TODO: lastUpdate correct instellen
    // lastUpdate = DateTime::Now(); (afhankelijk van jouw implementatie)
        float yaw = imu.GetCurrentYaw();
    //printf("Yaw: %.2f\n", yaw);
    return sensorsUpdated;
}

float SensorHub::GetSpeedLeft() const {
    return encoderLeft.GetLinVelocity();
}

float SensorHub::GetAngVelocity() const{
    return imu.GetAngVelocity();
}
float SensorHub::GetSpeedRight() const {
    return encoderRight.GetLinVelocity();
}

float SensorHub::GetCurrentYaw()const{
    return imu.GetCurrentYaw();
}

DateTime SensorHub::GetLastUpdate() const {
    return lastUpdate;
}

// In SensorHub.cpp
float SensorHub::GetEncoderYaw() {
    float DistanceL = encoderLeft.GetDistanceMm();
    float DistanceR = encoderRight.GetDistanceMm();

    // Bereken de hoek in radialen en zet om naar graden
    float encoderYawRad = (DistanceR - DistanceL) / 219.0f;
    currentEncoderYaw = encoderYawRad * (180.0f / M_PI);
    return currentEncoderYaw;
}