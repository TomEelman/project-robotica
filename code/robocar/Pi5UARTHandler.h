#pragma once

#include <string>
#include <vector>

// ═══════════════════════════════════════════════════════════════
//  Pi5UARTHandler  –  alle UART-communicatie met de Pico
//
//  Gebruik:
//    Pi5UARTHandler uart("/dev/ttyAMA10");
//    if (!uart.Open()) { /* fout */ }
//
//    auto enc = uart.LeesEncoder();
//    auto imu = uart.LeesIMU();
//    uart.Close();
// ═══════════════════════════════════════════════════════════════

struct EncoderData {
    float speedLinks;    // mm/s
    float afstandLinks;  // mm
    float speedRechts;   // mm/s
    float afstandRechts; // mm
    bool  geldig;
};

struct IMUData {
    float yawGraden;     // graden
    float yawRadialen;   // radialen (automatisch berekend)
    float hoeksnelheid;  // rad/s
    bool  geldig;
};

class Pi5UARTHandler {
public:
    explicit Pi5UARTHandler(const std::string& port, int baudrate = 115200);
    ~Pi5UARTHandler();

    // Verbinding openen/sluiten
    bool Open();
    void Close();
    bool IsOpen() const;

    // Sensor-uitlezing
    EncoderData LeesEncoder();
    IMUData     LeesIMU();

    // Laag-niveau: stuur een commando en wacht op antwoord
    std::string StuurVerzoek(const std::string& sensor);

    // Geeft de poort-naam terug (voor logging)
    const std::string& GetPort() const { return port; }

private:
    std::string port;
    int         baudrate;
    int         fd;   // file descriptor

    bool ConfigureerPoort();
};