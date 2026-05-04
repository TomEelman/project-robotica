#include "Pi5UARTHandler.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <sstream>

// ═══════════════════════════════════════════════════════════════════
//  Constructor / Destructor
// ═══════════════════════════════════════════════════════════════════

Pi5UARTHandler::Pi5UARTHandler(const std::string& port, int baudrate)
    : port(port), baudrate(baudrate), fd(-1)
{
}

Pi5UARTHandler::~Pi5UARTHandler() {
    Close();
}

// ═══════════════════════════════════════════════════════════════════
//  Open
// ═══════════════════════════════════════════════════════════════════

bool Pi5UARTHandler::Open() {
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror(("Pi5UARTHandler: open " + port).c_str());
        return false;
    }
    if (!ConfigureerPoort()) {
        Close();
        return false;
    }
    std::cout << "UART geopend op " << port << "\n";
    return true;
}

// ═══════════════════════════════════════════════════════════════════
//  ConfigureerPoort  –  115200 8N1, geen flow control
// ═══════════════════════════════════════════════════════════════════

bool Pi5UARTHandler::ConfigureerPoort() {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        perror("Pi5UARTHandler: tcgetattr");
        return false;
    }

    // Baudrate
    speed_t baud = B115200;
    if (baudrate == 9600)   baud = B9600;
    if (baudrate == 57600)  baud = B57600;
    if (baudrate == 115200) baud = B115200;

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // 8N1
    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |=  (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    // Geen software flow control, geen speciale byte-verwerking
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag  = 0;

    // Lees-timeout: 200 ms (VTIME = tiende van een seconde)
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 2;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Pi5UARTHandler: tcsetattr");
        return false;
    }
    return true;
}

// ═══════════════════════════════════════════════════════════════════
//  Close
// ═══════════════════════════════════════════════════════════════════

void Pi5UARTHandler::Close() {
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}

bool Pi5UARTHandler::IsOpen() const {
    return fd >= 0;
}

// ═══════════════════════════════════════════════════════════════════
//  StuurVerzoek  –  "GET:SENSOR\n"  →  antwoord als string
// ═══════════════════════════════════════════════════════════════════

std::string Pi5UARTHandler::StuurVerzoek(const std::string& sensor) {
    if (!IsOpen()) return "";

    tcflush(fd, TCIOFLUSH);

    std::string verzoek = "GET:" + sensor + "\n";
    write(fd, verzoek.c_str(), verzoek.size());

    std::string antwoord;
    char c;
    while (read(fd, &c, 1) > 0) {
        if (c == '\n') break;
        antwoord += c;
    }
    return antwoord;
}

// ═══════════════════════════════════════════════════════════════════
//  LeesEncoder
//  Verwacht antwoord:  "ENCODER:speedL,distL,speedR,distR\n"
// ═══════════════════════════════════════════════════════════════════

EncoderData Pi5UARTHandler::LeesEncoder() {
    EncoderData data{};
    data.geldig = false;

    std::string raw = StuurVerzoek("ENCODER");
    if (raw.empty()) return data;

    // Parse "ENCODER:v0,v1,v2,v3"
    size_t kol = raw.find(':');
    if (kol == std::string::npos) return data;
    if (raw.substr(0, kol) != "ENCODER") return data;

    std::vector<float> waarden;
    std::stringstream ss(raw.substr(kol + 1));
    std::string token;
    while (std::getline(ss, token, ',')) {
        try { waarden.push_back(std::stof(token)); }
        catch (...) {}
    }

    if (waarden.size() < 4) return data;

    data.speedLinks    = waarden[0];
    data.afstandLinks  = waarden[1];
    data.speedRechts   = waarden[2];
    data.afstandRechts = waarden[3];
    data.geldig        = true;
    return data;
}

// ═══════════════════════════════════════════════════════════════════
//  LeesIMU
//  Verwacht antwoord:  "IMU:yawDeg,omegaRad\n"
// ═══════════════════════════════════════════════════════════════════

IMUData Pi5UARTHandler::LeesIMU() {
    IMUData data{};
    data.geldig = false;

    std::string raw = StuurVerzoek("IMU");
    if (raw.empty()) return data;

    size_t kol = raw.find(':');
    if (kol == std::string::npos) return data;
    if (raw.substr(0, kol) != "IMU") return data;

    std::vector<float> waarden;
    std::stringstream ss(raw.substr(kol + 1));
    std::string token;
    while (std::getline(ss, token, ',')) {
        try { waarden.push_back(std::stof(token)); }
        catch (...) {}
    }

    if (waarden.size() < 1) return data;

    constexpr float DEG2RAD = 3.14159265f / 180.0f;

    data.yawGraden    = waarden[0];
    data.yawRadialen  = waarden[0] * DEG2RAD;
    data.hoeksnelheid = (waarden.size() >= 2) ? waarden[1] : 0.0f;
    data.geldig       = true;
    return data;
}