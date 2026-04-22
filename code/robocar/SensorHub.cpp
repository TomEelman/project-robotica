    #include "SensorHub.h"
    SensorHub::SensorHub(int encLeft, int encLeftRes,
                        int encRight, int encRightRes,
                        int imuSDAPin, int imuSCLPin)
        : encoderLeft(encLeft, encLeftRes),
        encoderRight(encRight, encRightRes),
        imu(imuSDAPin, imuSCLPin, 0x28),
        sensorsUpdated(false)
    {
    }


        bool SensorHub::UpdateSensors() {
            bool imuOk     = imu.Update();
            bool leftOk    = encoderLeft.Update();
            bool rightOk   = encoderRight.Update();

            sensorsUpdated = imuOk && leftOk && rightOk;

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
