#ifndef SENSORHUB_H
#define SENSORHUB_H

#include "IMU.h"
#include "Encoder.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

class SensorHub {
public:
    SensorHub(int encLeftRes, int encRightRes,
              int imuSdaPin,  int imuSclPin);

    bool UpdateSensors();

    float GetSpeedLeft()     const;
    float GetDistanceLeft()  const;
    float GetSpeedRight()    const;
    float GetDistanceRight() const;
    float GetCurrentYaw()    const;
    float GetAngVelocity()   const;

    bool HasFreshLeft()  const { return encoderLeft.HasFreshData();  }
    bool HasFreshRight() const { return encoderRight.HasFreshData(); }
    void ConsumeFreshFlags()
    {
        encoderLeft.ConsumeFreshFlag();
        encoderRight.ConsumeFreshFlag();
    }

private:
    IMU     imu;
    Encoder encoderLeft;
    Encoder encoderRight;
    bool sensorsUpdated;
};

#endif