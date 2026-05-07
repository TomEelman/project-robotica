#include "Pi5UARTHandler.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <cstdlib>

Pi5UARTHandler::Pi5UARTHandler(const std::string& port, int baudrate)
    : port(port), baudrate(baudrate), fd(-1)
{
}

Pi5UARTHandler::~Pi5UARTHandler() {
    Close();
}

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

bool Pi5UARTHandler::ConfigureerPoort() {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        perror("Pi5UARTHandler: tcgetattr");
        return false;
    }

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

    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag  = 0;

    // 200 ms lees-timeout
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 2;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Pi5UARTHandler: tcsetattr");
        return false;
    }
    return true;
}

void Pi5UARTHandler::Close() {
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}

bool Pi5UARTHandler::IsOpen() const {
    return fd >= 0;
}

// ─────────────────────────────────────────────────────────────────
//  LeesRegel  –  lees bytes tot '\n' of timeout
// ─────────────────────────────────────────────────────────────────
std::string Pi5UARTHandler::LeesRegel() {
    std::string antwoord;
    char c;
    while (read(fd, &c, 1) > 0) {
        if (c == '\n') break;
        if (c != '\r') antwoord += c;
    }
    return antwoord;
}

// ─────────────────────────────────────────────────────────────────
//  StuurRegel  –  stuur string zonder op antwoord te wachten
// ─────────────────────────────────────────────────────────────────
void Pi5UARTHandler::StuurRegel(const std::string& regel) {
    if (!IsOpen()) return;
    write(fd, regel.c_str(), regel.size());
}

// ─────────────────────────────────────────────────────────────────
//  StuurVerzoek  –  stuur GET-verzoek en wacht op antwoord-regel
//  (ongewijzigd t.o.v. origineel, nu intern gebruik van LeesRegel)
// ─────────────────────────────────────────────────────────────────
std::string Pi5UARTHandler::StuurVerzoek(const std::string& sensor) {
    if (!IsOpen()) return "";
    tcflush(fd, TCIOFLUSH);

    std::string verzoek = "GET:" + sensor + "\n";
    write(fd, verzoek.c_str(), verzoek.size());

    return LeesRegel();
}

// ─────────────────────────────────────────────────────────────────
//  StuurCommand  –  stuur "CMD:lin,ang\n" en wacht op "ACK:..."
// ─────────────────────────────────────────────────────────────────
CommandAck Pi5UARTHandler::StuurCommand(float lin, float ang) {
    CommandAck result{false, 0.0f, 0.0f};
    if (!IsOpen()) return result;

    // Formatteer het commando
    char buf[64];
    snprintf(buf, sizeof(buf), "CMD:%.3f,%.3f\n", lin, ang);

    tcflush(fd, TCIOFLUSH);
    write(fd, buf, strlen(buf));

    // Wacht op ACK of ERR van de Pico
    std::string antwoord = LeesRegel();
    if (antwoord.empty()) {
        std::cerr << "Pi5UARTHandler: geen ACK ontvangen voor CMD\n";
        return result;
    }

    return ParseAck(antwoord);
}

// ─────────────────────────────────────────────────────────────────
//  StuurStop  –  snelkoppeling voor een directe stop
// ─────────────────────────────────────────────────────────────────
CommandAck Pi5UARTHandler::StuurStop() {
    return StuurCommand(0.0f, 0.0f);
}

bool Pi5UARTHandler::RebootPico() {
    if (!IsOpen()) return false;

    tcflush(fd, TCIOFLUSH);
    StuurRegel("REBOOT\n");

    // Wacht op ACK:REBOOT
    std::string antwoord = LeesRegel();
    if (antwoord == "ACK:REBOOT") {
        std::cout << "Pi5UARTHandler: Pico reboot bevestigd\n";
        return true;
    }

    std::cerr << "Pi5UARTHandler: geen reboot ACK ontvangen (kreeg: " 
              << antwoord << ")\n";
    return false;
}

// ─────────────────────────────────────────────────────────────────
//  ParseAck  –  verwerk "ACK:lin,ang" van de Pico
// ─────────────────────────────────────────────────────────────────
CommandAck Pi5UARTHandler::ParseAck(const std::string& antwoord) {
    CommandAck result{false, 0.0f, 0.0f};

    // Moet beginnen met "ACK:"
    if (antwoord.substr(0, 4) != "ACK:") return result;

    const char* data = antwoord.c_str() + 4;
    char* endPtr     = nullptr;

    result.lin = strtof(data, &endPtr);
    if (endPtr == data || *endPtr != ',') return result;

    const char* angStart = endPtr + 1;
    result.ang  = strtof(angStart, &endPtr);
    result.geldig = (endPtr != angStart);
    return result;
}

// ─────────────────────────────────────────────────────────────────
//  LeesEncoder  (ongewijzigd)
// ─────────────────────────────────────────────────────────────────
EncoderData Pi5UARTHandler::LeesEncoder() {
    EncoderData data{};
    data.geldig = false;

    std::string raw = StuurVerzoek("ENCODER");
    if (raw.empty()) return data;

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

// ─────────────────────────────────────────────────────────────────
//  LeesIMU  (ongewijzigd)
// ─────────────────────────────────────────────────────────────────
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