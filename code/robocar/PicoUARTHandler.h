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

// Hoe vaak sensor-data gepusht wordt (ms)
static constexpr uint32_t PUSH_INTERVAL_MS   = 50;

// Watchdog: geen CMD binnen deze tijd → stop
static constexpr uint32_t PICO_CMD_TIMEOUT_MS = 500;

class PicoUARTHandler {
public:
    explicit PicoUARTHandler(SensorHub& sensorHub);

    // Initialiseer UART0 en installeer IRQ.
    void Init();

    // Aanroepen elke loop-iteratie (~10 ms).
    // - Verwerkt inkomend CMD als de IRQ er één klaar heeft gezet
    // - Pusht DATA naar Pi5 als PUSH_INTERVAL_MS verstreken is
    // - Geeft stop als CMD-timeout verlopen is
    // Geeft true als er een nieuw DriveCommand beschikbaar is.
    bool Tick(DriveCommand& out);

    static void UartRxIrqHandler();

private:
    SensorHub& sensorHub;

    DriveCommand pendingCmd;
    bool         hasPendingCmd;
    uint32_t     lastCmdTimeMs;
    uint32_t     lastPushMs;

    static volatile char rx_buffer[64];
    static volatile int  rx_pos;
    static volatile bool bericht_klaar;
    static PicoUARTHandler* instance;

    void HandleLine(const char* line);
    void PushData();
    bool ParseCmd(const char* line, float& lin, float& ang);
    void Send(const char* msg);
};