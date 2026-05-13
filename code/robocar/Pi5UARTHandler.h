#pragma once

#include <string>

// ═══════════════════════════════════════════════════════════════
//  Pi5UARTHandler  —  Pi5-kant UART, push-model
//
//  De Pico stuurt automatisch elke 50ms:
//    "DATA:encL,encR,yaw,omega\n"
//
//  Pi5 stuurt:
//    "CMD:lin,ang\n"   →  DriveCommand naar Pico
//    "REBOOT\n"        →  Pico herstart
//
//  LeesData() is non-blocking: leest wat er in de buffer zit
//  en slaat de laatste waarden op. Geen GET-requests meer,
//  geen 200ms timeout per sensor.
// ═══════════════════════════════════════════════════════════════

struct SensorData {
    float speedLinks;    // mm/s
    float speedRechts;   // mm/s
    float yawGraden;     // graden
    float hoeksnelheid;  // graden/s
    bool  geldig;        // true = minstens één DATA-pakket ontvangen
};

struct CommandAck {
    bool  geldig;
    float lin;
    float ang;
};

class Pi5UARTHandler {
public:
    explicit Pi5UARTHandler(const std::string& port, int baudrate = 115200);
    ~Pi5UARTHandler();

    bool Open();
    void Close();
    bool IsOpen() const;

    // Non-blocking: leest alle beschikbare DATA-regels uit de buffer,
    // slaat de laatste op. Geeft true als er nieuwe data was.
    bool LeesData();

    // Geeft de laatste ontvangen sensorwaarden terug.
    SensorData GetSensorData() const { return lastData; }

    // Stuur een DriveCommand naar de Pico (fire-and-forget, geen ACK-wait).
    void StuurCommand(float lin, float ang);
    void StuurStop();
    bool RebootPico();

    const std::string& GetPort() const { return port; }

private:
    std::string port;
    int         baudrate;
    int         fd;

    SensorData  lastData{};
    char        lineBuffer[128];
    int         linePos;

    bool ConfigureerPoort();
    bool ParseDataRegel(const char* regel);
};