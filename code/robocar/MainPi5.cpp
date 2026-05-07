#include "MainPi5.h"
#include "LIDAR.h"
#include "Mapper.h"
#include "Localisation.h"
#include "Pi5UARTHandler.h"
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <string>
#include <limits>

static volatile bool running = true;
static void onSignal(int) { running = false; }


// ════════════════════════════════════════════════════════════════
//  Menu
// ════════════════════════════════════════════════════════════════

enum class MenuKeuze {
    MAPPEN,
    PICO_COMMUNICATIE,
    STOPPEN
};

static void PrintMenu() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════╗\n";
    std::cout << "║        ROBOT CONTROLLER  Pi5         ║\n";
    std::cout << "╠══════════════════════════════════════╣\n";
    std::cout << "║  1. Mappen                           ║\n";
    std::cout << "║  2. Pico communiceren                ║\n";
    std::cout << "║  3. Stoppen                          ║\n";
    std::cout << "╚══════════════════════════════════════╝\n";
    std::cout << "Keuze: ";
}

static MenuKeuze VraagMenuKeuze() {
    while (true) {
        PrintMenu();
        std::string invoer;
        std::getline(std::cin, invoer);

        if (invoer == "1") return MenuKeuze::MAPPEN;
        if (invoer == "2") return MenuKeuze::PICO_COMMUNICATIE;
        if (invoer == "3") return MenuKeuze::STOPPEN;

        std::cout << "Ongeldige keuze. Probeer opnieuw.\n";
    }
}


// ════════════════════════════════════════════════════════════════
//  Init helpers
// ════════════════════════════════════════════════════════════════

static bool initUartHandler(Pi5UARTHandler& uart) {
    if (!uart.Open()) {
        std::cerr << "UART niet beschikbaar - rijdt zonder sensor-correctie.\n";
        return false;
    }
    return true;
}

static bool initLidar(LIDAR& lidar) {
    if (!lidar.Connect()) {
        std::cerr << "Kan niet verbinden met LIDAR. Programma gestopt.\n";
        return false;
    }
    return true;
}


// ════════════════════════════════════════════════════════════════
//  Modus 1: Mappen
// ════════════════════════════════════════════════════════════════

static int RunMappen(Pi5UARTHandler& uart, LIDAR& lidar) {
    Localisation loc(0.235f);
    Mapper       mapper(1200, 1200, 0.04f);   // 15×15 m @ 4 cm/cel

    constexpr float DT      = 0.1f;
    constexpr long  LOOP_US = 100000;
    int scanCount = 0;

    std::cout << "\033[2J";
    std::cout << "Mapping gestart. Motor spin-up...\n";
    usleep(1200000);

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        // 1. Encoder → EKF predict
        EncoderData enc = uart.LeesEncoder();
        if (enc.geldig)
            loc.Predict(enc.speedLinks / 1000.0f, enc.speedRechts / 1000.0f, DT);

        // 2. IMU → EKF update
        IMUData imu = uart.LeesIMU();
        if (imu.geldig)
            loc.UpdateIMU(imu.yawRadialen, DT);

        // 3. LIDAR scan ophalen
        if (!lidar.Update()) {
            usleep(200000);
            continue;
        }

        float ranges[360], angles[360];
        for (int angle = 0; angle < 360; ++angle) {
            ranges[angle] = lidar.GetDistance(angle).distance;
            angles[angle] = static_cast<float>(angle);
        }

        // 4. Kaart bijwerken
        Position pos(loc.GetX(), loc.GetY(), loc.GetTheta());
        mapper.Update(ranges, angles, 360, pos);
        ++scanCount;

        // 5. Status printen
        printf("Scans: %4d  Dekking: %2d%%  Pos: (%.2f, %.2f)  theta: %.1fdeg\n",
               scanCount, mapper.GetCoverage(),
               loc.GetX(), loc.GetY(),
               loc.GetTheta() * 180.0f / 3.14159265f);

        mapper.PrintMap(loc.GetX(), loc.GetY(), scanCount, mapper.GetCoverage());

        // 6. Loop timing
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::steady_clock::now() - tStart).count();
        long rest = LOOP_US - elapsed;
        if (rest > 0) usleep(static_cast<useconds_t>(rest));
    }

    std::cout << "\nStoppen...\n";
    lidar.Disconnect();

    if (mapper.SaveDebugMap("kaart.pgm"))
        printf("Kaart opgeslagen: kaart.pgm  (dekking: %d%%)\n", mapper.GetCoverage());

    return 0;
}


// ════════════════════════════════════════════════════════════════
//  Modus 2: Pico communicatie
//  Submenu: sturen of ontvangen
// ════════════════════════════════════════════════════════════════

static void PrintPicoMenu() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════╗\n";
    std::cout << "║        PICO COMMUNICATIE             ║\n";
    std::cout << "╠══════════════════════════════════════╣\n";
    std::cout << "║  1. DriveCommand sturen              ║\n";
    std::cout << "║  2. Sensordata ontvangen (live)      ║\n";
    std::cout << "║  3. Noodstop sturen                  ║\n";
    std::cout << "║  4. Terug naar hoofdmenu             ║\n";
    std::cout << "╚══════════════════════════════════════╝\n";
    std::cout << "Keuze: ";
}

static void RunStuurCommand(Pi5UARTHandler& uart) {
    float lin = 0.0f, ang = 0.0f;

    std::cout << "Lineaire snelheid (mm/s, positief=vooruit): ";
    std::cin >> lin;
    std::cout << "Hoeksnelheid (deg/s, positief=rechts):      ";
    std::cin >> ang;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    CommandAck ack = uart.StuurCommand(lin, ang);
    if (ack.geldig)
        printf("  ACK ontvangen: lin=%.2f  ang=%.2f\n", ack.lin, ack.ang);
    else
        std::cout << "  Geen ACK van Pico.\n";
}

static void RunLeesLive(Pi5UARTHandler& uart) {
    std::cout << "Live sensordata (Ctrl+C om te stoppen)...\n\n";

    running = true;
    while (running) {
        EncoderData enc = uart.LeesEncoder();
        IMUData     imu = uart.LeesIMU();

        if (enc.geldig)
            printf("ENC  sL=%7.2f mm/s  dL=%7.1f mm  sR=%7.2f mm/s  dR=%7.1f mm\n",
                   enc.speedLinks, enc.afstandLinks,
                   enc.speedRechts, enc.afstandRechts);
        else
            std::cout << "ENC  [geen data]\n";

        if (imu.geldig)
            printf("IMU  yaw=%7.2f deg  omega=%6.3f rad/s\n",
                   imu.yawGraden, imu.hoeksnelheid);
        else
            std::cout << "IMU  [geen data]\n";

        std::cout << "─────────────────────────────────────────\n";
        usleep(200000);   // 5 Hz
    }
    running = true;   // reset voor volgende modus
}

static int RunPicoCommunicatie(Pi5UARTHandler& uart) {
    while (true) {
        PrintPicoMenu();
        std::string invoer;
        std::getline(std::cin, invoer);

        if (invoer == "1") {
            RunStuurCommand(uart);
        } else if (invoer == "2") {
            RunLeesLive(uart);
        } else if (invoer == "3") {
            CommandAck ack = uart.StuurStop();
            std::cout << (ack.geldig ? "  Noodstop ACK ontvangen.\n"
                                     : "  Noodstop verstuurd (geen ACK).\n");
        } else if (invoer == "4") {
            break;
        } else {
            std::cout << "Ongeldige keuze.\n";
        }
    }
    return 0;
}


// ════════════════════════════════════════════════════════════════
//  main
// ════════════════════════════════════════════════════════════════

int main() {
    signal(SIGINT, onSignal);

    Pi5UARTHandler uart("/dev/ttyAMA10");
    LIDAR          lidar("/dev/ttyUSB0", 460800);

    // UART altijd proberen te openen; LIDAR alleen bij mappen
    initUartHandler(uart);

    MenuKeuze keuze = VraagMenuKeuze();

    switch (keuze) {
        case MenuKeuze::MAPPEN:
            if (!initLidar(lidar)) return 1;
            return RunMappen(uart, lidar);

        case MenuKeuze::PICO_COMMUNICATIE:
            return RunPicoCommunicatie(uart);

        case MenuKeuze::STOPPEN:
            std::cout << "Programma afgesloten.\n";
            return 0;
    }

    return 0;
}