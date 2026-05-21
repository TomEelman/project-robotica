#include "SensorHub.h"
#include <string.h>
#include <cstdio>

volatile char  SensorHub::rxBuffer[UART_BUFFER_LEN] = {0};
volatile int   SensorHub::rxPos       = 0;
volatile bool  SensorHub::messageReady = false;
SensorHub*     SensorHub::instance    = nullptr;

SensorHub::SensorHub(int encLeft,   int encLeftRes,
                     int encRight,  int encRightRes,
                     int imuSdaPin, int imuSclPin)
    : encoderLeft (encLeft,  encLeftRes),
      encoderRight(encRight, encRightRes),
      imu(imuSdaPin, imuSclPin, 0x28),
      sensorsUpdated(false)
{
    instance = this;
}

void SensorHub::InitUart()
{
    uart_init(SENSOR_UART, SENSOR_BAUD);
    gpio_set_function(SENSOR_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(SENSOR_UART_RX, GPIO_FUNC_UART);
    uart_set_hw_flow(SENSOR_UART, false, false);
    uart_set_format(SENSOR_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(SENSOR_UART, false); // byte-by-byte IRQ, no buffering delay

    int irqNum = (SENSOR_UART == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(irqNum, uartRxIrqHandler);
    irq_set_enabled(irqNum, true);
    uart_set_irq_enables(SENSOR_UART, true, false);
}

void SensorHub::uartRxIrqHandler()
{
    while (uart_is_readable(SENSOR_UART)) {
        char c = uart_getc(SENSOR_UART);

        if (c == '\r') continue;

        if (c == '\n') {
            rxBuffer[rxPos] = '\0';
            rxPos = 0;
            messageReady = true;
        } else if (rxPos < UART_BUFFER_LEN - 1) {
            rxBuffer[rxPos++] = c;
        }
    }
}

void SensorHub::HandleUart()
{
    if (!messageReady) return;

    char request[UART_BUFFER_LEN];
    strncpy(request, (const char*)rxBuffer, sizeof(request));
    messageReady = false;

    processRequest(request);
}

void SensorHub::processRequest(const char* request)
{
    UpdateSensors();

    char response[128];

    if (strcmp(request, "GET:ENCODER") == 0) {
        snprintf(response, sizeof(response), "ENCODER:%.2f,%.2f,%.2f,%.2f\n",
            GetSpeedLeft(),  GetDistanceLeft(),
            GetSpeedRight(), GetDistanceRight());

    } else if (strcmp(request, "GET:IMU") == 0) {
        snprintf(response, sizeof(response), "IMU:%.4f,%.4f\n",
            GetCurrentYaw(), GetAngVelocity());

    } else {
        snprintf(response, sizeof(response), "ERROR:UNKNOWN\n");
    }

    sendResponse(response);
}

void SensorHub::sendResponse(const char* response)
{
    uart_puts(SENSOR_UART, response);
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
DateTime SensorHub::GetLastUpdate()    const { return lastUpdate;                    }