#pragma once

// ═══════════════════════════════════════════════════════════════════
//  UARTCommandHandler  –  Pico-kant UART communicatie
//
//  Verwerkt twee soorten berichten van de RPi5:
//    1) "GET:ENCODER\n"  → antwoord met "ENCODER:sL,dL,sR,dR\n"
//    2) "GET:IMU\n"      → antwoord met "IMU:yawDeg,omega\n"
//    3) "CMD:lin,ang\n"  → DriveCommand doorsturen naar robot
//
//  Gebruik in main-loop (Pico):
//    UARTCommandHandler uart(robot, sensorHub);
//    uart.Poll();   // elke 10ms aanroepen
// ═══════════════════════════════════════════════════════════════════

#include "Robot.h"
#include "SensorHub.h"
#include "DriveCommand.h"

// Timeout in ms: als er langer geen CMD binnenkomt → stop robot
static constexpr uint32_t CMD_TIMEOUT_MS = 500;

class UARTCommandHandler {
public:
    UARTCommandHandler(Robot& robot, SensorHub& sensorHub);

    // Aanroepen elke loop-iteratie (~10ms).
    // Leest UART, dispatcht GET-verzoeken en CMD-commando's.
    void Poll();

    // Geeft true als er recentelijk een geldig CMD ontvangen is
    bool HasActiveCommand() const;

    // Geeft het laatste ontvangen DriveCommand terug
    DriveCommand GetLastCommand() const;

private:
    Robot&      robot;
    SensorHub&  sensorHub;

    char        rxBuf[128];
    int         rxLen;

    DriveCommand lastCmd;
    bool         hasCmd;
    uint32_t     lastCmdTimeMs;

    // Parse een volledige regel en reageer
    void HandleLine(const char* line);

    // Stuur sensor-antwoorden terug over UART0
    void SendEncoder();
    void SendIMU();

    // Parse "CMD:lin,ang"
    bool ParseCmd(const char* line, float& lin, float& ang);
};