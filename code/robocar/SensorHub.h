#ifndef SENSORHUB_H
#define SENSORHUB_H

#include "IMU.h"
#include "Encoder.h"
#include "DateTime.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define UART_ID      uart0
#define BAUD_RATE    115200
#define UART_TX_PIN  0
#define UART_RX_PIN  1

class SensorHub {
private:
    IMU     imu;
    Encoder encoderLeft;
    Encoder encoderRight;
    DateTime lastUpdate;
    bool sensorsUpdated;

    // ── UART ──────────────────────────────────────
    static volatile char rx_buffer[64];
    static volatile int  rx_pos;
    static volatile bool bericht_klaar;
    static SensorHub*    instance;       // Voor de static IRQ handler

    static void UartRxIrqHandler();
    void        VerwerkVerzoek(const char* verzoek);
    void        StuurAntwoord(const char* antwoord);

public:
    SensorHub(int encLeft, int encLeftRes,
              int encRight, int encRightRes,
              int imuSDAPin, int imuSCLPin);

    void InitUart();         // Initialiseer UART + IRQ
    void HandleUart();       // Aanroepen in main loop: checkt de flag

    bool  UpdateSensors();
    float GetSpeedLeft()     const;
    float GetDistanceLeft()  const;
    float GetSpeedRight()    const;
    float GetDistanceRight() const;
    float GetCurrentYaw()    const;
    float GetAngVelocity()   const;
    DateTime GetLastUpdate() const;
};

#endif