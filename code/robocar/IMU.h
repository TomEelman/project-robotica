#ifndef IMU_H
#define IMU_H

class IMU {
public:
    IMU(int sdaPin, int sclPin, int i2cAddress);

    bool Update();

    float GetLinVelocity()      const;
    float GetAngVelocity()      const;
    float GetCurrentYaw()       const;
    bool  GetCalibrationStatus() const;

    void SetI2CAddress(int address);
    int  GetI2CAddress() const;
    int  GetSDAPin()     const;
    int  GetSCLPin()     const;

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