#ifndef SENSORHUB_H
#define SENSORHUB_H

#include "IMU.h"
#include "Encoder.h"
#include "DateTime.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define SENSOR_UART       uart0 // default uart for 
static constexpr uint         SENSOR_BAUD     = 115200;
static constexpr int          SENSOR_UART_TX  = 0;
static constexpr int          SENSOR_UART_RX  = 1;
static constexpr int          UART_BUFFER_LEN = 64;

class SensorHub {
public:
    SensorHub(int encLeft,    int encLeftRes,
              int encRight,   int encRightRes,
              int imuSdaPin,  int imuSclPin);

    void InitUart();
    void HandleUart();

    bool UpdateSensors();

    float GetSpeedLeft()     const;
    float GetDistanceLeft()  const;
    float GetSpeedRight()    const;
    float GetDistanceRight() const;
    float GetCurrentYaw()    const;
    float GetAngVelocity()   const;
    DateTime GetLastUpdate() const;

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
    DateTime lastUpdate;
    bool sensorsUpdated;

    static volatile char rxBuffer[UART_BUFFER_LEN];
    static volatile int  rxPos;
    static volatile bool messageReady;
    static SensorHub*    instance;

    static void uartRxIrqHandler();
    void        processRequest(const char* request);
    void        sendResponse(const char* response);
};

#endif