#ifndef LIDAR_H
#define LIDAR_H

#include "hardware/uart.h"

#define SCAN_SIZE 360

class LIDAR {

private:
    uart_inst_t* uartPort;
    int          baudRate;
    int          maxRange;
    int          minRange;
    int          scanData[SCAN_SIZE];
    int          rotationSpeed;
    int          currentAngle;
    int          nextAngle;
    bool         updated;

public:
    LIDAR(uart_inst_t* uartPort, int baudRate = 460800);

    bool Update();

    bool IsObjectInRange(int minAngle, int maxAngle, int threshold) const;

    void ApplyMotionCorrection(float currentYaw);
};

#endif