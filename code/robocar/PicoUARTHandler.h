#pragma once

// ═══════════════════════════════════════════════════════════════════
//  PicoUARTHandler  –  Pico-kant, eigenaar van UART0 + IRQ
//
//  Vervangt de InitUart() + IRQ die voorheen in SensorHub zaten.
//  SensorHub blijft ongewijzigd en weet niets van UART.
//
//  Verwerkt:
//    "GET:ENCODER\n"  → stuurt "ENCODER:sL,dL,sR,dR\n" terug
//    "GET:IMU\n"      → stuurt "IMU:yaw,omega\n" terug
//    "CMD:lin,ang\n"  → slaat DriveCommand op, stuurt "ACK:lin,ang\n"
//
//  Gebruik in main (Pico):
//    PicoUARTHandler uartHandler(sensorHub);
//    uartHandler.Init();           // één keer, vóór de loop
//
//    // In de loop:
//    DriveCommand cmd(0,0);
//    if (uartHandler.ConsumePendingCmd(cmd))
//        robot.Execute(cmd);
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

// Timeout: geen CMD ontvangen binnen deze tijd → stop-commando
static constexpr uint32_t PICO_CMD_TIMEOUT_MS = 500;

class PicoUARTHandler {
public:
    explicit PicoUARTHandler(SensorHub& sensorHub);

    // Initialiseer UART0 en installeer de IRQ.
    // Aanroepen vóór de main-loop, NA constructie van SensorHub.
    void Init();

    // Aanroepen elke loop-iteratie (~10 ms).
    // Verwerkt een bericht als de IRQ er één klaar heeft gezet.
    // Geeft een stop-commando als de CMD-timeout verstreken is.
    // Geeft true + vult 'out' als er actie nodig is.
    bool ConsumePendingCmd(DriveCommand& out);

    // Statische IRQ-handler (public: vereist door irq_set_exclusive_handler)
    static void UartRxIrqHandler();

private:
    SensorHub& sensorHub;

    // Gepend commando
    DriveCommand pendingCmd;
    bool         hasPendingCmd;
    uint32_t     lastCmdTimeMs;

    // IRQ-gedeelde ontvangstbuffer (static zodat de IRQ erbij kan)
    static volatile char rx_buffer[64];
    static volatile int  rx_pos;
    static volatile bool bericht_klaar;
    static PicoUARTHandler* instance;

    // Verwerk één volledige regel
    void HandleLine(const char* line);

    // Stuur sensor-antwoorden
    void SendEncoder();
    void SendIMU();

    // Parse "CMD:lin,ang"
    bool ParseCmd(const char* line, float& lin, float& ang);

    // Stuur een string terug over UART
    void Send(const char* msg);
};