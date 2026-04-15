#include "SensorHub.h"
 
SensorHub::SensorHub(int encLeft, int encLeftRes,
                     int encRight, int encRightRes,
                     int imuSDAPin, int imuSCLPin,
                     uart_inst_t* lidarUart, int lidarBaud)
    : encoderLeft(encLeft, encLeftRes),
      encoderRight(encRight, encRightRes),
      imu(imuSDAPin, imuSCLPin, 0x28),
      lidar(lidarUart, lidarBaud),
      sensorsUpdated(false)
{
}


bool SensorHub::UpdateSensors() {
    bool imuOk     = imu.Update();
    bool leftOk    = encoderLeft.Update();
    bool rightOk   = encoderRight.Update();
    bool lidarOk   = lidar.Update();

    sensorsUpdated = imuOk && leftOk && rightOk && lidarOk;

    // TODO: set lastUpdate once a DateTime source is available
    // lastUpdate = DateTime::Now();

    return sensorsUpdated;
}

float SensorHub::GetSpeedLeft() const {
    return encoderLeft.GetLinVelocity();
}

float SensorHub::GetDistanceLeft() const {
    return encoderLeft.GetDistanceMm();
}

float SensorHub::GetSpeedRight() const {
    return encoderRight.GetLinVelocity();
}
float SensorHub::GetDistanceRight() const {
    return encoderRight.GetDistanceMm();
}


float SensorHub::GetCurrentYaw() const {
    return imu.GetCurrentYaw();
}
float SensorHub::GetAngVelocity()const{
    return imu.GetAngVelocity();
}

DateTime SensorHub::GetLastUpdate() const {
    return lastUpdate;
}

bool SensorHub::IsLidarObjectInRange(int minAngle, int maxAngle, int threshold) const {
    return lidar.IsObjectInRange(minAngle, maxAngle, threshold);
}