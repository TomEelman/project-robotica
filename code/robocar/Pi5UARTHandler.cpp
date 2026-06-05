#include "Pi5UARTHandler.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iostream>

Pi5UARTHandler::Pi5UARTHandler(const std::string& port, int baudrate)
    : port(port), baudrate(baudrate), fd(-1), linePos(0)
{
    memset(lineBuffer, 0, sizeof(lineBuffer));
    lastData = {};
}

Pi5UARTHandler::~Pi5UARTHandler() { Close(); }

bool Pi5UARTHandler::Open() {
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror(("Pi5UARTHandler: open " + port).c_str());
        return false;
    }
    if (!ConfigureerPoort()) { Close(); return false; }
    // Flush oude rommel uit de buffer
    tcflush(fd, TCIOFLUSH);
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
    if (baudrate ==  9600) baud = B9600;
    if (baudrate == 57600) baud = B57600;

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |=  (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag  = 0;

    // Non-blocking lezen
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Pi5UARTHandler: tcsetattr");
        return false;
    }
    return true;
}

void Pi5UARTHandler::Close() {
    if (fd >= 0) { close(fd); fd = -1; }
}

bool Pi5UARTHandler::IsOpen() const { return fd >= 0; }

// Non-blocking. Leest byte voor byte, bouwt regels op.
// Herkent DATA: pakketten. Logt alles anders naar stderr.
bool Pi5UARTHandler::LeesData() {
    if (!IsOpen()) return false;

    bool hadData = false;
    char c;

    while (read(fd, &c, 1) == 1) {
        if (c == '\r') continue;

        if (c == '\n') {
            lineBuffer[linePos] = '\0';
            int regelLen = linePos;  // bewaar lengte vÃ³Ã³r reset
            linePos = 0;
            if (regelLen > 0) {
                if (ParseDataRegel(lineBuffer)) {
                    hadData = true;
                } else if (strncmp(lineBuffer, "ACK:", 4) == 0) {
                    // ACK:OK:lin=...,ang=... en ACK:STOP komen elke 100ms
                } else if (regelLen > 2) {
                    // Onbekende regels loggen (ERR:, debug prints van Pico, etc.)
                    fprintf(stderr, "[Pico] %s\n", lineBuffer);
                }
            }
        } else if (linePos < (int)sizeof(lineBuffer) - 1) {
            lineBuffer[linePos++] = c;
        } else {
            // Buffer vol zonder newline sync verloren, reset
            linePos = 0;
        }
    }

    return hadData;
}

bool Pi5UARTHandler::ParseDataRegel(const char* regel) {
    if (strncmp(regel, "DATA:", 5) != 0) return false;

    const char* p = regel + 5;
    char* end;

    float encL  = strtof(p, &end); if (end == p || *end != ',') return false; p = end + 1;
    float encR  = strtof(p, &end); if (end == p || *end != ',') return false; p = end + 1;
    float yaw   = strtof(p, &end); if (end == p || *end != ',') return false; p = end + 1;
    float omega = strtof(p, &end); if (end == p)                return false;

    // Sanity check, gooi onmogelijke waarden weg
    if (encL < -5000.0f || encL > 5000.0f) return false;
    if (encR < -5000.0f || encR > 5000.0f) return false;
    if (yaw  <  -360.0f || yaw  >  360.0f) return false;

    lastData.speedLinks   = encL;
    lastData.speedRechts  = encR;
    lastData.yawGraden    = yaw;
    lastData.hoeksnelheid = omega;
    lastData.geldig       = true;
    return true;
}


void Pi5UARTHandler::StuurCommand(float lin, float ang) {
    if (!IsOpen()) return;
    char buf[48];
    snprintf(buf, sizeof(buf), "CMD:%.1f,%.1f\n", lin, ang);
    write(fd, buf, strlen(buf));
}

void Pi5UARTHandler::StuurStop() {
    if (!IsOpen()) return;
    const char* stop = "STOP\n";
    write(fd, stop, strlen(stop));
    usleep(5000);
    StuurCommand(0.0f, 0.0f);
}

bool Pi5UARTHandler::RebootPico() {
    if (!IsOpen()) return false;
    tcflush(fd, TCIOFLUSH);
    const char* msg = "REBOOT\n";
    write(fd, msg, strlen(msg));
    usleep(200000);

    char buf[64];
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    if (n > 0) {
        buf[n] = '\0';
        if (strstr(buf, "ACK:REBOOT")) {
            std::cout << "Pico reboot bevestigd\n";
            return true;
        }
    }
    std::cerr << "Pi5UARTHandler: geen reboot ACK\n";
    return false;
}