#ifndef IMU_H
#define IMU_H

class IMU {
public:
    IMU(int sdaPin, int sclPin, int i2cAddress);

    bool Update();
    float GetAngVelocity()      const;
    float GetCurrentYaw()       const;
    bool  GetCalibrationStatus() const;


private:
    int   sdaPin;
    int   sclPin;
    int   i2cAddress;

    float linearVelocity;
    float angularVelocity;
    float currentYaw;
    bool  calibrated;
};

#endif