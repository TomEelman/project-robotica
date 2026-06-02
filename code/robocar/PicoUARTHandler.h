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
//
//  Richtingswissel-guard:
//    De wielencoders zijn single-channel en meten alleen magnitude,
//    nooit richting. De robot mag daarom niet van rijrichting wisselen
//    terwijl hij nog beweegt — anders raakt het commando-teken (dat de
//    Pi5 gebruikt om de encoders een teken te geven) uit sync met de
//    werkelijke beweging, wat de kaart laat uitsmeren. Bij een omkering
//    vooruit<->achteruit stopt de Pico eerst tot de wielen echt stilstaan
//    en geeft dan pas het nieuwe commando vrij. Niet-blokkerend.
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
static constexpr uint32_t PUSH_INTERVAL_MS    = 50;

// Watchdog: geen CMD binnen deze tijd → stop
static constexpr uint32_t PICO_CMD_TIMEOUT_MS = 500;

// Richtingswissel-guard: wielsnelheid (mm/s) waaronder we de robot als
// "praktisch stil" beschouwen, en de maximale tijd dat we op stilstand
// wachten voordat we het nieuwe commando toch forceren.
static constexpr float    REVERSAL_STILL_MM_S = 15.0f;
static constexpr uint32_t REVERSAL_MAX_MS     = 800;

class PicoUARTHandler {
public:
    explicit PicoUARTHandler(SensorHub& sensorHub);

    // Initialiseer UART0 en installeer IRQ.
    void Init();

    // Aanroepen elke loop-iteratie (~10 ms).
    // - Verwerkt inkomend CMD als de IRQ er één klaar heeft gezet
    // - Pusht DATA naar Pi5 als PUSH_INTERVAL_MS verstreken is
    // - Houdt een richtingswissel vast tot de wielen stilstaan
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

    // ── Richtingswissel-guard ──────────────────────────────────────
    bool         reversalPending;   // wacht op stilstand voor richtingswissel
    DriveCommand reversalCmd;       // commando dat na stilstand volgt
    uint32_t     reversalStartMs;   // starttijd voor de veiligheidstimeout
    float        lastLinearSign;    // teken laatst doorgegeven commando (-1/0/+1)

    static volatile char rxBuffer[128];
    static volatile int  rxPos;
    static volatile bool messageReady;
    static PicoUARTHandler* instance;

    void handleLine(const char* line);
    void pushData();
    bool parseCmd(const char* line, float& lin, float& ang);
    void send(const char* msg);
};