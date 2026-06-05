#pragma once

// ═══════════════════════════════════════════════════════════════════
//  PicoUARTHandler  —  Pico-kant UART, push-model
//
//  Protocol (Pico → Pi5):
//    "DATA:encL,encR,yaw,omega\n"   elke PUSH_INTERVAL_MS ms
//
//  Protocol (Pi5 → Pico):
//    "CMD:lin,ang\n"   → DriveCommand uitvoeren + ACK terugsturen
//    "REBOOT\n"        → watchdog reset
//
//  De GET:ENCODER / GET:IMU berichten zijn vervallen.
//  De Pi5 hoeft niets meer te vragen — data stroomt automatisch.
// ═══════════════════════════════════════════════════════════════════

#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "SensorHub.h"
#include "DriveCommand.h"

#define PICO_UART_ID     uart0
#define PICO_UART_BAUD   115200
#define PICO_UART_TX_PIN 0
#define PICO_UART_RX_PIN 1

// how much sensor data is pushed (ms)
static constexpr uint32_t PUSH_INTERVAL_MS    = 50;

// Watchdog: no cmd within this time
static constexpr uint32_t PICO_CMD_TIMEOUT_MS = 500;

class PicoUARTHandler {
public:
    explicit PicoUARTHandler(SensorHub& sensorHub);

    // init UART0 and IRQ.
    void Init();

    // every loop iterate (~10 ms).
    // - handles CMD if IRQ has one set
    // - Pusht DATA to Pi5 if PUSH_INTERVAL_MS is done
    // - gives stop when CMD-timeout is done
    // gives true when new drivecommand is available
    bool Tick(DriveCommand& out);

    static void UartRxIrqHandler();

private:
    SensorHub& sensorHub;

    DriveCommand pendingCmd;
    bool         hasPendingCmd;
    uint32_t     lastCmdTimeMs;
    uint32_t     lastPushMs;

    static volatile char rxBuffer[128];
    static volatile int  rxPos;
    static volatile bool messageReady;
    static PicoUARTHandler* instance;

    void handleLine(const char* line);
    void pushData();
    bool parseCmd(const char* line, float& lin, float& ang);
    void send(const char* msg);
};