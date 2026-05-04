#include "MainPi5.h"
#include "LIDAR.h"
#include "Mapper.h"
#include "Localisation.h"
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <cstdio>
#include <cmath>

static volatile bool running = true;
static void onSignal(int) { running = false; }


bool initUartHandler(Pi5UARTHandler& uart) {
    if (!uart.Open()) {
        std::cerr << "UART niet beschikbaar - rijdt zonder sensor-correctie.\n";
        return false;
    }
    return true;
}

bool initLidar(LIDAR& lidar) {
    if (!lidar.Connect()) {
        std::cerr << "Kan niet verbinden met LIDAR. Programma gestopt.\n";
        return false;
    }
    return true;
}

bool initLocalisation(Localisation& loc) {
    (void)loc;
    return true;
}

bool initMapper(Mapper& mapper) {
    (void)mapper;
    return true;
}

int main() {
    signal(SIGINT, onSignal);

    // Objecten aanmaken
    Pi5UARTHandler uart("/dev/ttyAMA10");
    LIDAR          lidar("/dev/ttyUSB0", 460800);
    Localisation   loc(0.235f);
    Mapper         mapper(300, 300, 0.05f);

    // Initialiseren
    initUartHandler(uart);
    if (!initLidar(lidar)) return 1;
    initLocalisation(loc);
    initMapper(mapper);

    constexpr float DT      = 0.1f;       // 10 Hz
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
            
            // Geen hoekconversie — LIDAR output direct gebruiken
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
    uart.Close();

    if (mapper.SaveDebugMap("kaart.pgm"))
        printf("Kaart opgeslagen: kaart.pgm  (dekking: %d%%)\n", mapper.GetCoverage());

    return 0;
}