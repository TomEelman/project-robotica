#include "MainPi5.h"
#include "LIDAR.h"

#include <iostream>
#include <csignal>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <sstream>

// ════════════════════════════════════════════════════════════
//  UART
// ════════════════════════════════════════════════════════════

int openSerial(const char* port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) { perror("open"); return -1; }

    termios tty{};
    tcgetattr(fd, &tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag  = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 20;   // 2 seconden timeout
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

std::string vraag_sensor(int fd, const std::string& sensor) {
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

// ════════════════════════════════════════════════════════════
//  Parser
// ════════════════════════════════════════════════════════════

SensorData parseResponse(const std::string& response) {
    SensorData data;

    size_t kolonPos = response.find(':');
    if (kolonPos == std::string::npos) return data;

    data.type = response.substr(0, kolonPos);

    std::stringstream ss(response.substr(kolonPos + 1));
    std::string token;
    while (std::getline(ss, token, ',')) {
        try { data.waarden.push_back(std::stof(token)); }
        catch (...) {}
    }
    return data;
}

void printSensorData(const SensorData& data) {
    if (data.type == "ENCODER" && data.waarden.size() == 4) {
        std::cout << "Speed Links:    " << data.waarden[0] << " mm/s\n";
        std::cout << "Afstand Links:  " << data.waarden[1] << " mm\n";
        std::cout << "Speed Rechts:   " << data.waarden[2] << " mm/s\n";
        std::cout << "Afstand Rechts: " << data.waarden[3] << " mm\n";

    } else if (data.type == "IMU" && data.waarden.size() == 2) {
        std::cout << "Yaw:            " << data.waarden[0] << " graden\n";
        std::cout << "Hoeksnelheid:   " << data.waarden[1] << " rad/s\n";

    } else if (data.type == "ERROR") {
        std::cout << "Pico fout: onbekend commando\n";

    } else {
        std::cout << "Onbekend antwoord: " << data.type << "\n";
    }
}

// ════════════════════════════════════════════════════════════
//  LIDAR scan
// ════════════════════════════════════════════════════════════

static volatile bool ctrl_c_pressed = false;
static void onSignal(int) { ctrl_c_pressed = true; }

void doLidarScan(const std::string& port, int baudrate) {
    ctrl_c_pressed = false;
    signal(SIGINT, onSignal);

    LIDAR lidar(port.c_str(), baudrate);

    if (!lidar.Connect()) {
        std::cerr << "Kan niet verbinden met LIDAR op " << port << "\n";
        return;
    }

    std::cout << "LIDAR verbonden. Scannen... (Ctrl+C om te stoppen)\n";

    while (!ctrl_c_pressed) {
        if (!lidar.Update()) {
            std::cerr << "LIDAR update mislukt\n";
            break;
        }

        std::cout << "\n--- LIDAR scan ---\n";
        for (int angle = 0; angle < 360; angle += 10) {
            LIDAR::ScanEntry entry = lidar.GetDistance(angle);
            if (entry.distance > 0.0f) {
                printf("  Hoek: %3d graden  |  Afstand: %.1f mm\n",
                       entry.angle, entry.distance);
            }
        }
    }

    std::cout << "\nStoppen, LIDAR loskoppelen...\n";
    lidar.Disconnect();
}

// ════════════════════════════════════════════════════════════
//  Main menu
// ════════════════════════════════════════════════════════════

int main() {
    int fd = openSerial("/dev/ttyAMA10");
    if (fd < 0) return 1;

    std::cout << "=== RoboCar RPi5 interface ===\n";

    while (true) {
        std::cout << "\nKies actie:\n";
        std::cout << "  1) Sensor opvragen  (ENCODER / IMU)\n";
        std::cout << "  2) LIDAR scan\n";
        std::cout << "  0) Stoppen\n";
        std::cout << "> ";

        std::string keuze;
        std::cin >> keuze;

        if (keuze == "0" || keuze == "stop") {
            break;

        } else if (keuze == "1" || keuze == "ENCODER" || keuze == "IMU") {
            // Als de gebruiker direct ENCODER of IMU typt, direct doorsturen
            std::string sensor = keuze;
            if (keuze == "1") {
                std::cout << "Welke sensor? (ENCODER / IMU): ";
                std::cin >> sensor;
            }

            std::string resultaat = vraag_sensor(fd, sensor);
            if (resultaat.empty()) {
                std::cout << "Timeout: geen antwoord van Pico\n";
            } else {
                printSensorData(parseResponse(resultaat));
            }

        } else if (keuze == "2" || keuze == "LIDAR") {
            doLidarScan("/dev/ttyUSB0", 460800);

        } else {
            std::cout << "Onbekende keuze: " << keuze << "\n";
        }
    }

    close(fd);
    std::cout << "Programma gestopt.\n";
    return 0;
}