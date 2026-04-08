#include "SensorHub.h"
#include <cstdio>
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
    printf("Yaw: %.2f\n", yaw);
    return sensorsUpdated;
}

float SensorHub::GetSpeedLeft() const {
    return encoderLeft.GetLinVelocity();
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