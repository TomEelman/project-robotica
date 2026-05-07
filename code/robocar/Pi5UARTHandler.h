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
//    // Sensordata opvragen (Pico stuurt terug)
//    auto enc = uart.LeesEncoder();
//    auto imu = uart.LeesIMU();
//
//    // DriveCommand sturen naar Pico
//    bool ok = uart.StuurCommand(200.0f, 0.0f);   // 200 mm/s, rechtdoor
//    bool ok = uart.StuurCommand(0.0f,  45.0f);   // ter plekke draaien
//    bool ok = uart.StuurStop();                  // noodstop
//
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

// Resultaat van StuurCommand
struct CommandAck {
    bool  geldig;        // true = Pico stuurde "ACK:lin,ang" terug
    float lin;           // bevestigde lineaire snelheid
    float ang;           // bevestigde hoeksnelheid
};

class Pi5UARTHandler {
public:
    explicit Pi5UARTHandler(const std::string& port, int baudrate = 115200);
    ~Pi5UARTHandler();

    // Verbinding openen/sluiten
    bool Open();
    void Close();
    bool IsOpen() const;

    // ── Sensor-uitlezing (GET-protocol, ongewijzigd) ─────────
    EncoderData LeesEncoder();
    IMUData     LeesIMU();

    // ── DriveCommands sturen (CMD-protocol) ──────────────────
    //
    //  StuurCommand(lin, ang)
    //    lin : lineaire snelheid in mm/s  (positief = vooruit)
    //    ang : hoeksnelheid in graden/s   (positief = rechts)
    //
    //  Geeft een CommandAck terug. ack.geldig == true als de
    //  Pico het commando bevestigde.
    CommandAck StuurCommand(float lin, float ang);

    //  Snelkoppeling voor een directe stop
    CommandAck StuurStop();

    // ── Laag-niveau ──────────────────────────────────────────
    //  Stuur een vrij verzoek en wacht op antwoord-regel
    std::string StuurVerzoek(const std::string& sensor);

    //  Stuur een string, wacht NIET op antwoord (fire-and-forget)
    void StuurRegel(const std::string& regel);

    //  Lees één antwoord-regel (blokkerend, timeout via VTIME)
    std::string LeesRegel();

    // Geeft de poort-naam terug (voor logging)
    const std::string& GetPort() const { return port; }

private:
    std::string port;
    int         baudrate;
    int         fd;

    bool ConfigureerPoort();

    // Parse "ACK:lin,ang" → CommandAck
    CommandAck ParseAck(const std::string& antwoord);
};