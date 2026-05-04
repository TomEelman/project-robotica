// #include "MainPi5.h"
// #include "LIDAR.h"
// #include "Mapper.h"
// #include "Localisation.h"

// #include <iostream>
// #include <csignal>
// #include <fcntl.h>
// #include <termios.h>
// #include <unistd.h>
// #include <string>
// #include <vector>
// #include <sstream>
// #include <cstdio>
// #include <cmath>
// #include <chrono>

// // ════════════════════════════════════════════════════════════════════
// //  Globale stop-vlag  (Ctrl+C)
// // ════════════════════════════════════════════════════════════════════

// static volatile bool ctrl_c_pressed = false;
// static void onSignal(int) { ctrl_c_pressed = true; }

// // ════════════════════════════════════════════════════════════════════
// //  UART helpers
// // ════════════════════════════════════════════════════════════════════

// int openSerial(const char* port) {
//     int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
//     if (fd < 0) { perror("open"); return -1; }

//     termios tty{};
//     tcgetattr(fd, &tty);
//     cfsetispeed(&tty, B115200);
//     cfsetospeed(&tty, B115200);
//     tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
//     tty.c_cflag |= (CLOCAL | CREAD);
//     tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
//     tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | ICRNL);
//     tty.c_oflag &= ~OPOST;
//     tty.c_lflag  = 0;
//     tty.c_cc[VMIN]  = 0;
//     tty.c_cc[VTIME] = 20;  // 200 ms timeout per byte
//     tcsetattr(fd, TCSANOW, &tty);
//     return fd;
// }

// std::string vraag_sensor(int fd, const std::string& sensor) {
//     tcflush(fd, TCIOFLUSH);
//     std::string verzoek = "GET:" + sensor + "\n";
//     write(fd, verzoek.c_str(), verzoek.size());

//     std::string antwoord;
//     char c;
//     while (read(fd, &c, 1) > 0) {
//         if (c == '\n') break;
//         antwoord += c;
//     }
//     return antwoord;
// }

// // ════════════════════════════════════════════════════════════════════
// //  Parser
// // ════════════════════════════════════════════════════════════════════

// SensorData parseResponse(const std::string& response) {
//     SensorData data;
//     size_t kolonPos = response.find(':');
//     if (kolonPos == std::string::npos) return data;

//     data.type = response.substr(0, kolonPos);
//     std::stringstream ss(response.substr(kolonPos + 1));
//     std::string token;
//     while (std::getline(ss, token, ',')) {
//         try { data.waarden.push_back(std::stof(token)); }
//         catch (...) {}
//     }
//     return data;
// }

// void printSensorData(const SensorData& data) {
//     if (data.type == "ENCODER" && data.waarden.size() == 4) {
//         std::cout << "Speed Links:    " << data.waarden[0] << " mm/s\n"
//                   << "Afstand Links:  " << data.waarden[1] << " mm\n"
//                   << "Speed Rechts:   " << data.waarden[2] << " mm/s\n"
//                   << "Afstand Rechts: " << data.waarden[3] << " mm\n";
//     } else if (data.type == "IMU" && data.waarden.size() == 2) {
//         std::cout << "Yaw:          " << data.waarden[0] << " graden\n"
//                   << "Hoeksnelheid: " << data.waarden[1] << " rad/s\n";
//     } else if (data.type == "ERROR") {
//         std::cout << "Pico fout: onbekend commando\n";
//     } else {
//         std::cout << "Onbekend antwoord: " << data.type << "\n";
//     }
// }

// // ════════════════════════════════════════════════════════════════════
// //  Optie 2: LIDAR-only raw scan
// // ════════════════════════════════════════════════════════════════════

// void doLidarScan(const std::string& port, int baudrate) {
//     ctrl_c_pressed = false;
//     signal(SIGINT, onSignal);

//     LIDAR lidar(port.c_str(), baudrate);
//     if (!lidar.Connect()) {
//         std::cerr << "Kan niet verbinden met LIDAR op " << port << "\n";
//         return;
//     }

//     std::cout << "LIDAR verbonden. Scannen... (Ctrl+C om te stoppen)\n";

//     while (!ctrl_c_pressed) {
//         if (!lidar.Update()) {
//             std::cerr << "LIDAR update mislukt\n";
//             break;
//         }
//         std::cout << "\n--- LIDAR scan ---\n";
//         for (int angle = 0; angle < 360; angle += 10) {
//             LIDAR::ScanEntry e = lidar.GetDistance(angle);
//             if (e.distance > 0.0f)
//                 printf("  Hoek: %3d graden  |  Afstand: %.1f mm\n",
//                        e.angle, e.distance);
//         }
//     }

//     std::cout << "\nStoppen, LIDAR loskoppelen...\n";
//     lidar.Disconnect();
// }

// // ════════════════════════════════════════════════════════════════════
// //  Sensor-uitlees helpers (Pico via UART)
// // ════════════════════════════════════════════════════════════════════

// // Leest encoder-snelheden (mm/s) uit de Pico.
// // Geeft false terug bij timeout of parse-fout.
// static bool leesEncoder(int fd, float& vLinks, float& vRechts) {
//     std::string raw = vraag_sensor(fd, "ENCODER");
//     if (raw.empty()) return false;

//     SensorData d = parseResponse(raw);
//     if (d.type != "ENCODER" || d.waarden.size() < 4) return false;

//     // waarden[0]=speedL(mm/s), [1]=distL, [2]=speedR(mm/s), [3]=distR
//     vLinks  = d.waarden[0];
//     vRechts = d.waarden[2];
//     return true;
// }

// // Leest IMU yaw (graden) en zet om naar radialen.
// // Geeft false bij timeout.
// static bool leesIMU(int fd, float& yawRad) {
//     std::string raw = vraag_sensor(fd, "IMU");
//     if (raw.empty()) return false;

//     SensorData d = parseResponse(raw);
//     if (d.type != "IMU" || d.waarden.size() < 1) return false;

//     constexpr float DEG2RAD = 3.14159265f / 180.0f;
//     yawRad = d.waarden[0] * DEG2RAD;
//     return true;
// }

// // ════════════════════════════════════════════════════════════════════
// //  ASCII-visualisatie van de kaart in de terminal
// //  Toont een 40×20 venster rondom de robotpositie.
// // ════════════════════════════════════════════════════════════════════

// static void printMapWindow(const GridMap& map,
//                             float robotX, float robotY)
// {
//     constexpr int WIN_W = 60;
//     constexpr int WIN_H = 25;

//     int rx, ry;
//     map.WorldToCell(robotX, robotY, rx, ry);

//     int startX = rx - WIN_W / 2;
//     int startY = ry - WIN_H / 2;

//     // Verplaats cursor naar begin (ANSI escape)
//     std::cout << "\033[H";
//     std::cout << "┌── Occupancy Map (" << WIN_W << "×" << WIN_H
//               << ") ── R=robot, #=bezet, .=vrij, ?=onbekend ──┐\n";

//     for (int cy = startY + WIN_H - 1; cy >= startY; --cy) {
//         std::cout << "│";
//         for (int cx = startX; cx < startX + WIN_W; ++cx) {
//             if (cx == rx && cy == ry) {
//                 std::cout << "\033[32mR\033[0m";  // groen
//             } else if (!map.InBounds(cx, cy)) {
//                 std::cout << ' ';
//             } else if (map.IsOccupied(cx, cy)) {
//                 std::cout << "\033[31m#\033[0m";  // rood
//             } else if (map.IsFree(cx, cy)) {
//                 std::cout << '.';
//             } else {
//                 std::cout << '?';
//             }
//         }
//         std::cout << "│\n";
//     }
//     std::cout << "└";
//     for (int i = 0; i < WIN_W; i++) std::cout << "─";
//     std::cout << "┘\n";
// }

// // ════════════════════════════════════════════════════════════════════
// //  Optie 3: Live mapping (EKF-localisation + GridMap + LIDAR)
// //
// //  Loop-structuur (20 Hz target):
// //    1. Lees encoder + IMU via UART van de Pico
// //    2. EKF predict (odometrie) + update (IMU)
// //    3. Lees LIDAR-scan
// //    4. Integreer scan in GridMap via Mapper
// //    5. Print ASCII-kaartvenster + stats
// // ════════════════════════════════════════════════════════════════════

// void doLiveMapping(const std::string& lidarPort, int lidarBaud, int serialFd) {
//     ctrl_c_pressed = false;
//     signal(SIGINT, onSignal);

//     // ── Initialisatie ────────────────────────────────────────────
//     // Kaart: 400×400 cellen @ 5 cm/cel = 20×20 meter
//     Mapper       mapper(400, 400, 0.05f);
//     // EKF: wielbasis van de RoboCar (pas aan indien nodig)
//     Localisation loc(0.235f);
//     // LIDAR
//     LIDAR        lidar(lidarPort.c_str(), lidarBaud);

//     if (!lidar.Connect()) {
//         std::cerr << "Kan niet verbinden met LIDAR op " << lidarPort << "\n";
//         return;
//     }

//     // Schermbuffer leegmaken voor ASCII-kaart
//     std::cout << "\033[2J";

//     std::cout << "Live mapping gestart. (Ctrl+C om te stoppen en kaart op te slaan)\n";
//     usleep(800000);  // 0.8 s voor motoropstart LIDAR

//     // ── Tijdsbeheer ──────────────────────────────────────────────
//     constexpr float LOOP_HZ  = 20.0f;
//     constexpr float DT        = 1.0f / LOOP_HZ;
//     constexpr long  LOOP_US   = static_cast<long>(DT * 1e6f);

//     int iteratie = 0;

//     // ── Hoofd-loop ───────────────────────────────────────────────
//     while (!ctrl_c_pressed) {
//         auto tStart = std::chrono::steady_clock::now();

//         // ── Stap 1: Encoder + IMU van de Pico ────────────────────
//         float vLinks = 0.0f, vRechts = 0.0f;
//         bool encoderOk = leesEncoder(serialFd, vLinks, vRechts);

//         float imuYaw = 0.0f;
//         bool imuOk = leesIMU(serialFd, imuYaw);

//         // ── Stap 2: EKF-lokalisatie ───────────────────────────────
//         // Encoder: mm/s → m/s voor de EKF
//         if (encoderOk) {
//             loc.Predict(vLinks / 1000.0f, vRechts / 1000.0f, DT);
//         }
//         if (imuOk) {
//             loc.UpdateIMU(imuYaw, DT);
//         }

//         float robotX     = loc.GetX();
//         float robotY     = loc.GetY();
//         float robotTheta = loc.GetTheta();

//         // ── Stap 3: LIDAR-scan ────────────────────────────────────
//         if (!lidar.Update()) {
//             std::cerr << "LIDAR update mislukt, opnieuw proberen...\n";
//             usleep(100000);
//             continue;
//         }

//         // Bouw ranges[] en angles[] arrays op
//         float ranges[360];
//         float angles[360];
//         for (int a = 0; a < 360; ++a) {
//             LIDAR::ScanEntry e = lidar.GetDistance(a);
//             ranges[a] = e.distance;               // mm
//             angles[a] = static_cast<float>(a);    // graden
//         }

//         // ── Stap 4: Kaart bijwerken ───────────────────────────────
//         Position pos(robotX, robotY, robotTheta);
//         mapper.Update(ranges, angles, 360, pos);

//         // ── Stap 5: ASCII-visualisatie (elke 5 iteraties) ─────────
//         if (iteratie % 5 == 0) {
//             printMapWindow(mapper.GetMap(), robotX, robotY);

//             // Status-regel onder de kaart
//             printf("Pos: (%.2f m, %.2f m)  θ=%.1f°  Dekking: %d%%  "
//                    "Encoder:%s  IMU:%s  [iter %d]\n",
//                    robotX, robotY,
//                    robotTheta * 180.0f / 3.14159265f,
//                    mapper.GetCoverage(),
//                    encoderOk ? "OK" : "--",
//                    imuOk     ? "OK" : "--",
//                    iteratie);

//             // Nabije obstakels melden
//             bool frontVrij = !lidar.IsObjectInRange(350, 360, 300.0f) &&
//                              !lidar.IsObjectInRange(0,   10,  300.0f);
//             bool achterVrij = !lidar.IsObjectInRange(170, 190, 200.0f);
//             printf("Obstakel voor: %s  |  achter: %s\n",
//                    frontVrij  ? "vrij " : "DICHT",
//                    achterVrij ? "vrij " : "DICHT");
//         }

//         ++iteratie;

//         // ── Loop-timing: slaap het resterende deel van de tijdslot ─
//         auto tEnd  = std::chrono::steady_clock::now();
//         long elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
//                            tEnd - tStart).count();
//         long remaining = LOOP_US - elapsed;
//         if (remaining > 0) usleep(static_cast<useconds_t>(remaining));
//     }

//     // ── Opruimen & kaart opslaan ─────────────────────────────────
//     std::cout << "\n\nStoppen — LIDAR loskoppelen...\n";
//     lidar.Disconnect();

//     std::string filename = "kaart.pgm";
//     if (mapper.SaveDebugMap(filename)) {
//         std::cout << "Kaart opgeslagen als '" << filename << "'\n";
//         std::cout << "Eindige dekking: " << mapper.GetCoverage() << "%\n";
//     } else {
//         std::cerr << "Kaart opslaan mislukt.\n";
//     }
// }

// // ════════════════════════════════════════════════════════════════════
// //  Main  –  interactief menu
// // ════════════════════════════════════════════════════════════════════

// int main() {
//     int fd = openSerial("/dev/ttyAMA10");
//     if (fd < 0) return 1;

//     std::cout << "=== RoboCar RPi5 interface ===\n";

//     while (true) {
//         std::cout << "\nKies actie:\n"
//                   << "  1) Sensor opvragen  (ENCODER / IMU)\n"
//                   << "  2) LIDAR scan       (raw poolcoördinaten)\n"
//                   << "  3) Live mapping     (EKF + GridMap + ASCII-kaart)\n"
//                   << "  0) Stoppen\n"
//                   << "> ";

//         std::string keuze;
//         std::cin >> keuze;

//         if (keuze == "0" || keuze == "stop") {
//             break;

//         } else if (keuze == "1" || keuze == "ENCODER" || keuze == "IMU") {
//             std::string sensor = keuze;
//             if (keuze == "1") {
//                 std::cout << "Welke sensor? (ENCODER / IMU): ";
//                 std::cin >> sensor;
//             }
//             std::string resultaat = vraag_sensor(fd, sensor);
//             if (resultaat.empty())
//                 std::cout << "Timeout: geen antwoord van Pico\n";
//             else
//                 printSensorData(parseResponse(resultaat));

//         } else if (keuze == "2" || keuze == "LIDAR") {
//             doLidarScan("/dev/ttyUSB0", 460800);

//         } else if (keuze == "3" || keuze == "MAP" || keuze == "map") {
//             doLiveMapping("/dev/ttyUSB0", 460800, fd);

//         } else {
//             std::cout << "Onbekende keuze: " << keuze << "\n";
//         }
//     }

//     close(fd);
//     std::cout << "Programma gestopt.\n";
//     return 0;
// }

#include "LIDAR.h"
#include "Mapper.h"
#include "Localisation.h"

#include <iostream>
#include <csignal>
#include <unistd.h>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <string>

// ════════════════════════════════════════════════════════════════════
//  Stationair testprogramma  –  alleen LIDAR nodig
//
//  De robot staat stil: encoder = 0, IMU = 0.
//  De LIDAR draait en de kaart wordt live bijgewerkt in de terminal.
//
//  Gebruik:
//    g++ -std=c++17 -I. -I~/rplidar_sdk/sdk/include \
//        LidarMapTest.cpp LIDAR.cpp Mapper.cpp GridMap.cpp \
//        -L~/rplidar_sdk/output/Linux/Release -lsl_lidar_sdk \
//        -lpthread -o lidar_test
//    ./lidar_test
// ════════════════════════════════════════════════════════════════════

static volatile bool stop = false;
static void onSignal(int) { stop = true; }

// ── ASCII-kaartvenster ───────────────────────────────────────────────
static void printMap(const GridMap& map, float robotX, float robotY,
                     int scanCount, int coverage)
{
    constexpr int WIN_W = 62;
    constexpr int WIN_H = 24;

    int rx, ry;
    map.WorldToCell(robotX, robotY, rx, ry);

    int startX = rx - WIN_W / 2;
    int startY = ry - WIN_H / 2;

    std::cout << "\033[H";  // cursor naar boven

    printf("╔══ LIDAR Occupancy Map  [scans: %4d]  dekking: %2d%% ══╗\n",
           scanCount, coverage);

    for (int cy = startY + WIN_H - 1; cy >= startY; --cy) {
        std::cout << "║";
        for (int cx = startX; cx < startX + WIN_W; ++cx) {
            if (cx == rx && cy == ry) {
                std::cout << "\033[92mR\033[0m";   // groen = robot
            } else if (!map.InBounds(cx, cy)) {
                std::cout << ' ';
            } else if (map.IsOccupied(cx, cy)) {
                std::cout << "\033[91m█\033[0m";   // rood = muur
            } else if (map.IsFree(cx, cy)) {
                std::cout << "\033[90m·\033[0m";   // grijs = vrij
            } else {
                std::cout << ' ';                  // onbekend = leeg
            }
        }
        std::cout << "║\n";
    }

    printf("╚══ R=robot  █=muur  ·=vrij  spatie=onbekend");
    for (int i = 0; i < WIN_W - 38; i++) std::cout << ' ';
    std::cout << "══╝\n";
    std::cout << "Ctrl+C om te stoppen en kaart op te slaan als kaart.pgm\n";
}

// ── Dichtstbijzijnde obstakel per kwadrant ────────────────────────────
static void printObstacles(const LIDAR& lidar) {
    // Zoek de minimale afstand in elk kwadrant (voor/achter/links/rechts)
    float minVoor    = 99999, minAchter = 99999;
    float minLinks   = 99999, minRechts = 99999;

    for (int a = 0; a < 360; ++a) {
        float d = lidar.GetDistance(a).distance;
        if (d <= 0.0f) continue;

        if      (a <= 30  || a >= 330) minVoor    = std::min(minVoor,    d);
        else if (a >= 150 && a <= 210) minAchter  = std::min(minAchter,  d);
        else if (a >  30  && a <  150) minLinks   = std::min(minLinks,   d);
        else                            minRechts  = std::min(minRechts,  d);
    }

    auto fmt = [](float v) -> std::string {
        if (v > 9000) return "  vrij ";
        char buf[16];
        snprintf(buf, sizeof(buf), "%5.0f mm", v);
        return buf;
    };

    printf("Obstakels — voor: %s  |  achter: %s  |  links: %s  |  rechts: %s\n",
           fmt(minVoor).c_str(), fmt(minAchter).c_str(),
           fmt(minLinks).c_str(), fmt(minRechts).c_str());
}

// ════════════════════════════════════════════════════════════════════
//  Main
// ════════════════════════════════════════════════════════════════════

int main() {
    signal(SIGINT, onSignal);

    // ── LIDAR verbinding ─────────────────────────────────────────
    const std::string port    = "/dev/ttyUSB0";
    const int         baud    = 460800;

    LIDAR lidar(port, baud);
    std::cout << "Verbinden met LIDAR op " << port << "...\n";

    if (!lidar.Connect()) {
        std::cerr << "Fout: kan niet verbinden. Controleer USB-poort.\n"
                  << "Beschikbare poorten: ls /dev/ttyUSB*\n";
        return 1;
    }
    std::cout << "LIDAR verbonden. Motor start... (wacht 1 s)\n";
    usleep(1200000);

    // ── Kaart: 300×300 cellen @ 5 cm = 15×15 meter ───────────────
    Mapper mapper(300, 300, 0.05f);

    // Robot staat stil in het midden, positie = (0, 0), theta = 0
    const float ROBOT_X     = 0.0f;
    const float ROBOT_Y     = 0.0f;
    const float ROBOT_THETA = 0.0f;
    Position fixedPos(ROBOT_X, ROBOT_Y, ROBOT_THETA);

    // ── Scherm leegmaken ─────────────────────────────────────────
    std::cout << "\033[2J";

    int  scanCount = 0;
    bool firstScan = true;

    constexpr long LOOP_US = 100000;  // 10 Hz (LIDAR is de bottleneck)

    // ── Hoofd-loop ───────────────────────────────────────────────
    while (!stop) {
        auto tStart = std::chrono::steady_clock::now();

        // 1. Lees scan
        if (!lidar.Update()) {
            std::cerr << "LIDAR update mislukt\n";
            usleep(200000);
            continue;
        }

        // 2. Bouw arrays op
        float ranges[360], angles[360];
        int   validPoints = 0;

        for (int a = 0; a < 360; ++a) {
            LIDAR::ScanEntry e = lidar.GetDistance(a);
            ranges[a] = e.distance;
            angles[a] = static_cast<float>(a);
            if (e.distance > 0.0f) ++validPoints;
        }

        // Sla lege scans over (motor nog niet op snelheid)
        if (validPoints < 10) {
            std::cout << "Wachten op stabiele scan... (" << validPoints << " punten)\n";
            usleep(300000);
            continue;
        }

        if (firstScan) {
            std::cout << "\033[2J";  // scherm leeg na eerste geldige scan
            firstScan = false;
        }

        // 3. Kaart bijwerken (robot staat stil)
        mapper.Update(ranges, angles, 360, fixedPos);
        ++scanCount;

        // 4. Visualisatie
        printMap(mapper.GetMap(), ROBOT_X, ROBOT_Y,
                 scanCount, mapper.GetCoverage());
        printObstacles(lidar);

        // 5. Timing
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::steady_clock::now() - tStart).count();
        long remaining = LOOP_US - elapsed;
        if (remaining > 0) usleep(static_cast<useconds_t>(remaining));
    }

    // ── Opruimen ─────────────────────────────────────────────────
    std::cout << "\n\nStoppen — LIDAR loskoppelen...\n";
    lidar.Disconnect();

    if (mapper.SaveDebugMap("kaart.pgm")) {
        std::cout << "Kaart opgeslagen als 'kaart.pgm'\n";
        std::cout << "Bekijk op Pi: eog kaart.pgm   of   gpicview kaart.pgm\n";
        std::cout << "Eindige dekking: " << mapper.GetCoverage() << "%\n";
    }

    return 0;
}