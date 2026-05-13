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
//
//    uart.StuurCommand(200.0f, 0.0f);   // 200 mm/s, rechtdoor
//    uart.StuurCommand(0.0f,  45.0f);   // ter plekke draaien
//    uart.StuurStop();                  // noodstop
//
//    uart.Close();
// ═══════════════════════════════════════════════════════════════

struct EncoderData {
    float speedLinks;     // mm/s
    float afstandLinks;   // mm
    float speedRechts;    // mm/s
    float afstandRechts;  // mm
    bool  geldig;
};

struct IMUData {
    float yawGraden;      // graden  (gebruik dit altijd)
    float hoeksnelheid;   // graden/s
    bool  geldig;
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

    EncoderData LeesEncoder();
    IMUData     LeesIMU();

    CommandAck StuurCommand(float lin, float ang);
    CommandAck StuurStop();
    bool       RebootPico();

    std::string StuurVerzoek(const std::string& sensor);
    void        StuurRegel(const std::string& regel);
    std::string LeesRegel();

    const std::string& GetPort() const { return port; }

private:
    std::string port;
    int         baudrate;
    int         fd;

    bool       ConfigureerPoort();
    CommandAck ParseAck(const std::string& antwoord);
};