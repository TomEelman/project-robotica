#include "SensorHub.h"
#include <string.h>
#include <cstdio>

SensorHub::SensorHub(int encLeftRes,int encRightRes,
                     int imuSdaPin, int imuSclPin)
    : encoderLeft (encLeftRes),
      encoderRight(encRightRes),
      imu(imuSdaPin, imuSclPin, 0x28),
      sensorsUpdated(false)
{
}


bool SensorHub::UpdateSensors()
{
    bool imuOk   = imu.Update();
    bool leftOk  = encoderLeft.Update();
    bool rightOk = encoderRight.Update();
    sensorsUpdated = imuOk && leftOk && rightOk;
    return sensorsUpdated;
}

float    SensorHub::GetSpeedLeft()     const { return encoderLeft.GetLinVelocity();  }
float    SensorHub::GetDistanceLeft()  const { return encoderLeft.GetDistanceMm();   }
float    SensorHub::GetSpeedRight()    const { return encoderRight.GetLinVelocity(); }
float    SensorHub::GetDistanceRight() const { return encoderRight.GetDistanceMm();  }
float    SensorHub::GetCurrentYaw()    const { return imu.GetCurrentYaw();           }
float    SensorHub::GetAngVelocity()   const { return imu.GetAngVelocity();          }
