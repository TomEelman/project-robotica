#include "MainPi5.h"
#include "LIDAR.h"
#include "Mapper.h"
#include "Localisation.h"
#include "Pi5UARTHandler.h"
#include <csignal>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <unistd.h>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <string>
#include <limits>

static volatile bool running = true;
static void onSignal(int) { running = false; }


// ════════════════════════════════════════════════════════════════
//  CommandKeepAlive
//
//  Achtergrond-thread die het actieve commando elke 200 ms opnieuw
//  stuurt. De Pico-timeout is 500 ms, dus zo blijft hij rijden.
//  Zodra je een nieuw commando geeft of stopt, wissel je gewoon
//  via SetCommand() of Stop().
// ════════════════════════════════════════════════════════════════

class CommandKeepAlive {
public:
    explicit CommandKeepAlive(Pi5UARTHandler& uart)
        : uart(uart), lin(0.0f), ang(0.0f), actief(false)
    {}

    // Zet een nieuw actief commando en begin met herhalen
    void SetCommand(float linearMmS, float angularDegS) {
        std::lock_guard<std::mutex> lk(mtx);
        lin    = linearMmS;
        ang    = angularDegS;
        actief = true;
    }

    // Stop het herhalen en stuur een expliciete stop naar de Pico
    void Stop() {
        {
            std::lock_guard<std::mutex> lk(mtx);
            lin    = 0.0f;
            ang    = 0.0f;
            actief = false;
        }
        uart.StuurStop();
    }

    bool IsActief() const { return actief.load(); }
    float GetLin()  const { return lin; }
    float GetAng()  const { return ang; }

    // Start de achtergrond-thread (aanroepen vóór de loop)
    void Start() {
        threadActief = true;
        worker = std::thread([this]() {
            while (threadActief) {
                {
                    std::lock_guard<std::mutex> lk(mtx);
                    if (actief)
                        uart.StuurCommand(lin, ang);
                }
                // Wacht 200 ms tussen herhalingen (ruim onder 500 ms timeout)
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        });
    }

    // Stop de thread netjes (aanroepen bij afsluiten)
    void Shutdown() {
        threadActief = false;
        if (worker.joinable()) worker.join();
    }

private:
    Pi5UARTHandler& uart;
    std::mutex      mtx;
    float           lin;
    float           ang;
    std::atomic<bool> actief;
    std::atomic<bool> threadActief{false};
    std::thread     worker;
};





// ════════════════════════════════════════════════════════════════
//  Init helpers
// ════════════════════════════════════════════════════════════════

static bool initUartHandler(Pi5UARTHandler& uart) {
    if (!uart.Open()) {
        std::cerr << "UART niet beschikbaar.\n";
        return false;
    }
    return true;
}

static bool initLidar(LIDAR& lidar) {
    if (!lidar.Connect()) {
        std::cerr << "Kan niet verbinden met LIDAR.\n";
        return false;
    }
    return true;
}


// ════════════════════════════════════════════════════════════════
//  Modus 1: Mappen (ongewijzigd)
// ════════════════════════════════════════════════════════════════

static int RunMappen(Pi5UARTHandler& uart, LIDAR& lidar) {
    Localisation loc(0.235f);
    Mapper       mapper(1200, 1200, 0.04f);

    constexpr float DT      = 0.1f;
    constexpr long  LOOP_US = 100000;
    int scanCount = 0;

    std::cout << "\033[2J";
    std::cout << "Mapping gestart. Motor spin-up...\n";
    usleep(1200000);

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        EncoderData enc = uart.LeesEncoder();
        if (enc.geldig)
            loc.Predict(enc.speedLinks / 1000.0f, enc.speedRechts / 1000.0f, DT);

        IMUData imu = uart.LeesIMU();
        if (imu.geldig)
            loc.UpdateIMU(imu.yawGraden, DT);

        if (!lidar.Update()) { usleep(200000); continue; }

        float ranges[360], angles[360];
        for (int angle = 0; angle < 360; ++angle) {
            ranges[angle] = lidar.GetDistance(angle).distance;
            angles[angle] = static_cast<float>(angle);
        }

        Position pos(loc.GetX(), loc.GetY(), loc.GetTheta());
        mapper.Update(ranges, angles, 360, pos);
        ++scanCount;

        printf("Scans: %4d  Dekking: %2d%%  Pos: (%.2f, %.2f)  theta: %.1fdeg\n",
               scanCount, mapper.GetCoverage(),
               loc.GetX(), loc.GetY(),
               loc.GetTheta());

        mapper.PrintMap(loc.GetX(), loc.GetY(), scanCount, mapper.GetCoverage());

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
// ════════════════════════════════════════════════════════════════

static void PrintPicoMenu(const CommandKeepAlive& ka) {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════╗\n";
    std::cout << "║        PICO COMMUNICATIE             ║\n";
    std::cout << "╠══════════════════════════════════════╣\n";

    if (ka.IsActief())
        printf("║  Actief: lin=%-6.1f  ang=%-10.1f║\n", ka.GetLin(), ka.GetAng());
    else
        std::cout << "║  Actief: GESTOPT                     ║\n";

    std::cout << "╠══════════════════════════════════════╣\n";
    std::cout << "║  1. DriveCommand sturen              ║\n";
    std::cout << "║  2. Sensordata ontvangen (live)      ║\n";
    std::cout << "║  3. Noodstop                         ║\n";
    std::cout << "║  4. Terug naar hoofdmenu             ║\n";
    std::cout << "╚══════════════════════════════════════╝\n";
    std::cout << "Keuze: ";
}

static void RunStuurCommand(CommandKeepAlive& ka) {
    float lin = 0.0f, ang = 0.0f;

    std::cout << "Lineaire snelheid (mm/s, positief=vooruit): ";
    std::cin >> lin;
    std::cout << "Hoeksnelheid (deg/s, positief=rechts):      ";
    std::cin >> ang;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    ka.SetCommand(lin, ang);
    printf("  Commando actief: lin=%.1f  ang=%.1f  (blijft rijden tot stop)\n",
           lin, ang);
}

static void RunLeesLive(Pi5UARTHandler& uart) {
    std::cout << "Live sensordata (druk Enter om te stoppen)...\n\n";

    // Non-blocking input check via aparte thread
    std::atomic<bool> stopLive{false};
    std::thread inputThread([&]() {
        std::string dummy;
        std::getline(std::cin, dummy);
        stopLive = true;
    });

    while (!stopLive) {
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
        usleep(200000);
    }

    if (inputThread.joinable()) inputThread.join();
}

static int RunPicoCommunicatie(Pi5UARTHandler& uart) {
    CommandKeepAlive ka(uart);
    ka.Start();

    while (true) {
        PrintPicoMenu(ka);
        std::string invoer;
        std::getline(std::cin, invoer);

        if (invoer == "1") {
            RunStuurCommand(ka);
        } else if (invoer == "2") {
            RunLeesLive(uart);
        } else if (invoer == "3") {
            ka.Stop();
            std::cout << "  Robot gestopt.\n";
        } else if (invoer == "4") {
            ka.Stop();
            ka.Shutdown();
            break;
        } else {
            std::cout << "Ongeldige keuze.\n";
        }
    }
    return 0;
}


// ════════════════════════════════════════════════════════════════
//  Modus 3: Rijden + Mappen + 90° draai + autonoom pathfinding
//
//  Fase 1  (0-10 s)    : lin=278 mm/s rechtdoor, kaart bouwen
//  Fase 2  (draai)     : 90° naar rechts op basis van IMU-yaw
//  Fase 3  (autonoom)  : PathPlanner + Navigator, herplan elke
//                        HERPLAN_SCANS scans, obstakelcheck vooruit
// ════════════════════════════════════════════════════════════════

#include "PathPlanner.h"
#include "Navigator.h"

// ── Hulpfunctie: kies een frontier-doel (verste FREE cel vanuit huidige pos) ──
// Als er geen frontier is, geeft hij de huidige positie terug (robot staat stil).
static Position KiesFrontierDoel(const Mapper& mapper, const Position& huidig)
{
    // Zoek de cel het verst weg die FREE is maar een UNKNOWN buur heeft
    // (klassieke frontier exploration). Eenvoudige variant: verste FREE cel.
    float bestDist = 0.0f;
    float bestWx   = huidig.GetX();
    float bestWy   = huidig.GetY();

    int W = mapper.GetMap().GetWidth();
    int H = mapper.GetMap().GetHeight();

    for (int cy = 0; cy < H; ++cy) {
        for (int cx = 0; cx < W; ++cx) {
            if (!mapper.GetMap().IsFree(cx, cy)) continue;

            // Moet minstens één UNKNOWN buur hebben (frontier voorwaarde)
            bool heeftOnbekendeBuur = false;
            const int dx[4] = {-1,1,0,0};
            const int dy[4] = {0,0,-1,1};
            for (int d = 0; d < 4; ++d) {
                int nx = cx + dx[d], ny = cy + dy[d];
                if (mapper.GetMap().InBounds(nx, ny) &&
                    mapper.GetMap().IsUnknown(nx, ny)) {
                    heeftOnbekendeBuur = true;
                    break;
                }
            }
            if (!heeftOnbekendeBuur) continue;

            float wx, wy;
            mapper.GetMap().CellToWorld(cx, cy, wx, wy);

            float dx2 = wx - huidig.GetX();
            float dy2 = wy - huidig.GetY();
            float dist = std::sqrt(dx2*dx2 + dy2*dy2);

            if (dist > bestDist) {
                bestDist = dist;
                bestWx   = wx;
                bestWy   = wy;
            }
        }
    }

    return Position(bestWx, bestWy, 0.0f);
}

// ── Hulpfunctie: obstakel recht vooruit? ────────────────────────────────────
// Kijkt in een kegel van ±HOEK_DEG graden voor de robot,
// binnen AFSTAND_MM mm. Geeft true als er een OCCUPIED cel in zit.
static bool ObstakelVooruit(const Mapper& mapper, const Position& pos,
                             float afstandMm = 600.0f, float hoekDeg = 30.0f)
{
    static constexpr float DEG2RAD = 3.14159265f / 180.0f;
    float thetaRad = pos.GetTheta() * DEG2RAD;  // graden → radialen voor cos/sin
    float hoekRad  = hoekDeg * DEG2RAD;

    // Scan in kleine stappen langs de kegel
    constexpr int STAPPEN = 20;
    for (int s = 1; s <= STAPPEN; ++s) {
        float r = afstandMm * static_cast<float>(s) / STAPPEN;

        // Linker- en rechterrand van de kegel + middenas
        for (float offset : {-hoekRad, 0.0f, hoekRad}) {
            float wx = pos.GetX() + r * std::cos(thetaRad + offset);
            float wy = pos.GetY() + r * std::sin(thetaRad + offset);

            int cx, cy;
            mapper.GetMap().WorldToCell(wx, wy, cx, cy);

            if (mapper.GetMap().InBounds(cx, cy) &&
                mapper.GetMap().IsOccupied(cx, cy))
                return true;
        }
    }
    return false;
}

static int RunRijdenEnMappen(Pi5UARTHandler& uart, LIDAR& lidar)
{
    Localisation loc(0.235f);
    Mapper       mapper(1200, 1200, 0.04f);   // 48×48 m @ 4 cm/cel

    constexpr float DT             = 0.1f;
    constexpr long  LOOP_US        = 100000;
    constexpr float LIN_SPEED      = 278.0f;
    constexpr float TURN_SPEED     =  50.0f;
    constexpr float TURN_TARGET    =  90.0f;
    constexpr int   RIJD_SECONDEN  =  10;
    constexpr int   HERPLAN_SCANS  =  15;

    enum class Fase { RECHTDOOR, DRAAIEN, AUTONOOM };
    Fase     fase              = Fase::RECHTDOOR;
    float    startYaw          = 0.0f;
    bool     yawGezet          = false;
    int      scanCount         = 0;
    int      scansSindsHerplan = 0;

    // Pathfinding objecten
    PathPlanner planner(mapper.GetMap());
    Navigator   navigator;
    bool        heeftPad   = false;

    // CommandKeepAlive voor de autonome fase
    CommandKeepAlive ka(uart);
    ka.Start();

    auto faseStart = std::chrono::steady_clock::now();

    std::cout << "\033[2J";
    printf("Fase 1: rechtdoor rijden (%.0f mm/s) gedurende %d s.\n",
           LIN_SPEED, RIJD_SECONDEN);
    printf("Fase 2: 90 graden draaien naar rechts.\n");
    printf("Fase 3: autonoom verkennen op basis van kaart.\n\n");
    usleep(1200000);   // LIDAR spin-up

    ka.SetCommand(LIN_SPEED, 0.0f);   // begin rijden

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        // ── 1. Encoder + IMU → lokalisatie ───────────────────────
        EncoderData enc = uart.LeesEncoder();
        if (enc.geldig)
            loc.Predict(enc.speedLinks / 1000.0f, enc.speedRechts / 1000.0f, DT);

        IMUData imu = uart.LeesIMU();
        if (imu.geldig)
            loc.UpdateIMU(imu.yawGraden, DT);

        Position pos(loc.GetX(), loc.GetY(), loc.GetTheta());

        // ── 2. LIDAR → kaart ─────────────────────────────────────
        bool nieuweScan = false;
        if (lidar.Update()) {
            float ranges[360], angles[360];
            for (int a = 0; a < 360; ++a) {
                ranges[a] = lidar.GetDistance(a).distance;
                angles[a] = static_cast<float>(a);
            }
            mapper.Update(ranges, angles, 360, pos);
            ++scanCount;
            ++scansSindsHerplan;
            nieuweScan = true;
        }

        // ── 3. Fase-logica ────────────────────────────────────────
        fase     = Fase::AUTONOOM;

        // --- Fase 1: rechtdoor ---
        // if (fase == Fase::RECHTDOOR) {
        //     auto verstreken = std::chrono::duration_cast<std::chrono::seconds>(
        //                           std::chrono::steady_clock::now() - faseStart).count();

        //     if (verstreken >= RIJD_SECONDEN) {
        //         printf("\nFase 2: 90 graden draaien...\n");
        //         ka.Stop();
        //         usleep(300000);
        //         fase     = Fase::DRAAIEN;
        //         yawGezet = false;
        //     }
        // }

        // --- Fase 2: draaien ---
        // else if (fase == Fase::DRAAIEN) {
        //     if (!yawGezet) {
        //         IMUData ref = uart.LeesIMU();
        //         startYaw    = ref.geldig ? ref.yawGraden : 0.0f;
        //         yawGezet    = true;
        //         ka.SetCommand(0.0f, TURN_SPEED);
        //         printf("Startyaw: %.1f graden\n", startYaw);
        //     }

        //     IMUData cur = uart.LeesIMU();
        //     if (cur.geldig) {
        //         float gedraaid = cur.yawGraden - startYaw;
        //         while (gedraaid >  180.0f) gedraaid -= 360.0f;
        //         while (gedraaid < -180.0f) gedraaid += 360.0f;

        //         printf("  Gedraaid: %.1f / %.0f graden\r", gedraaid, TURN_TARGET);
        //         fflush(stdout);

        //         if (gedraaid >= TURN_TARGET) {
        //             ka.Stop();
        //             usleep(300000);
        //             printf("\nDraai klaar. Autonome fase gestart.\n");
        //             scansSindsHerplan = HERPLAN_SCANS;
        //             fase = Fase::AUTONOOM;
        //         }
        //     }
        // }

        // --- Fase 3: autonoom pathfinding ---
        if (fase == Fase::AUTONOOM) {

            // ── 3a. Obstakel recht vooruit? Herplan direct ────────
            if (ObstakelVooruit(mapper, pos)) {
                printf("\nObstakel gedetecteerd! Herplan...\n");
                ka.Stop();
                usleep(200000);
                heeftPad          = false;
                scansSindsHerplan = HERPLAN_SCANS;
            }

            // ── 3b. Herplan elke HERPLAN_SCANS scans ─────────────
            if (nieuweScan && scansSindsHerplan >= HERPLAN_SCANS) {
                scansSindsHerplan = 0;

                // Debug: robotpositie in wereldcoördinaten en gridcel
                int robotCx = 0, robotCy = 0;
                mapper.GetMap().WorldToCell(pos.GetX(), pos.GetY(), robotCx, robotCy);
                bool robotCelVrij = mapper.GetMap().InBounds(robotCx, robotCy) &&
                                    mapper.GetMap().IsFree(robotCx, robotCy);
                printf("[AUTO] pos=(%.0f,%.0f)mm  cel=(%d,%d)  vrij=%s  dekking=%d%%\n",
                       pos.GetX(), pos.GetY(), robotCx, robotCy,
                       robotCelVrij ? "ja" : "NEE", mapper.GetCoverage());

                Position doel = KiesFrontierDoel(mapper, pos);

                if (doel.GetX() == pos.GetX() && doel.GetY() == pos.GetY()) {
                    printf("[AUTO] Geen frontier gevonden — dekking=%d%%\n",
                           mapper.GetCoverage());
                    // Nog niet klaar: te weinig kaart — rijd even vooruit en herplan
                    ka.SetCommand(LIN_SPEED, 0.0f);
                    scansSindsHerplan = HERPLAN_SCANS;  // herplan volgende scan
                } else {
                    printf("[AUTO] Frontier doel=(%.0f,%.0f)mm\n",
                           doel.GetX(), doel.GetY());

                    Path pad = planner.PlanPath(pos, doel, mapper.GetMap());

                    if (!pad.IsEmpty()) {
                        navigator.SetPath(pad);
                        heeftPad = true;
                        printf("[AUTO] Pad gevonden: %d waypoints\n", pad.GetSize());
                    } else {
                        printf("[AUTO] Geen pad naar frontier — rijd vooruit\n");
                        heeftPad = false;
                        // Geen pad: rijd gewoon vooruit i.p.v. rondjes draaien
                        ka.SetCommand(LIN_SPEED, 0.0f);
                        scansSindsHerplan = HERPLAN_SCANS;
                    }
                }
            }

            // ── 3c. Navigator → DriveCommand → keepalive ─────────
            if (heeftPad && !navigator.IsFinished()) {
                navigator.Update(pos);
                DriveCommand cmd = navigator.GetNextCommand(pos);
                ka.SetCommand(cmd.GetLinVelocity(), cmd.GetAngVelocity());
            } else if (heeftPad && navigator.IsFinished()) {
                // Waypoint bereikt, wacht op volgende herplan
                printf("[AUTO] Doel bereikt, wacht op volgende scan...\n");
                heeftPad = false;
                ka.Stop();
            }
            // Geen else meer voor draaien — vooruit rijden wordt hierboven gezet
        }

        // ── 4. Status printen ─────────────────────────────────────
        if (fase == Fase::RECHTDOOR) {
            auto verstreken = std::chrono::duration_cast<std::chrono::seconds>(
                                  std::chrono::steady_clock::now() - faseStart).count();
            printf("Scans:%4d  Dekking:%2d%%  Pos:(%.0f,%.0f)mm  theta:%.1f deg  t:%lds/%ds\n",
                   scanCount, mapper.GetCoverage(),
                   pos.GetX(), pos.GetY(),
                   loc.GetTheta(),
                   verstreken, RIJD_SECONDEN);
            mapper.PrintMap(pos.GetX(), pos.GetY(), scanCount, mapper.GetCoverage());
        } else if (fase == Fase::AUTONOOM && nieuweScan) {
            printf("Scans:%4d  Dekking:%2d%%  Pos:(%.0f,%.0f)mm  theta:%.1f deg  CMD:%.0f/%.0f\n",
                   scanCount, mapper.GetCoverage(),
                   pos.GetX(), pos.GetY(),
                   loc.GetTheta(),
                   ka.GetLin(), ka.GetAng());
            mapper.PrintMap(pos.GetX(), pos.GetY(), scanCount, mapper.GetCoverage());
        }

        // ── 5. Loop timing ────────────────────────────────────────
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::steady_clock::now() - tStart).count();
        long rest = LOOP_US - elapsed;
        if (rest > 0) usleep(static_cast<useconds_t>(rest));
    }

    ka.Stop();
    ka.Shutdown();
    lidar.Disconnect();

    if (mapper.SaveDebugMap("kaart.pgm"))
        printf("Kaart opgeslagen: kaart.pgm  (dekking: %d%%)\n", mapper.GetCoverage());

    return 0;
}


// ════════════════════════════════════════════════════════════════
//  main
// ════════════════════════════════════════════════════════════════

enum class MenuKeuze { MAPPEN, PICO_COMMUNICATIE, RIJDEN_EN_MAPPEN, STOPPEN };

static void PrintMenu() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════╗\n";
    std::cout << "║        ROBOT CONTROLLER  Pi5         ║\n";
    std::cout << "╠══════════════════════════════════════╣\n";
    std::cout << "║  1. Mappen                           ║\n";
    std::cout << "║  2. Pico communiceren                ║\n";
    std::cout << "║  3. Rijden + Mappen (10s + 90° draai)║\n";
    std::cout << "║  4. Stoppen                          ║\n";
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
        if (invoer == "3") return MenuKeuze::RIJDEN_EN_MAPPEN;
        if (invoer == "4") return MenuKeuze::STOPPEN;
        std::cout << "Ongeldige keuze. Probeer opnieuw.\n";
    }
}

int main() {
    signal(SIGINT, onSignal);

    Pi5UARTHandler uart("/dev/ttyAMA10");
    LIDAR          lidar("/dev/ttyUSB0", 460800);

    initUartHandler(uart);

    MenuKeuze keuze = VraagMenuKeuze();

    uart.RebootPico();

    switch (keuze) {
        case MenuKeuze::MAPPEN:
            if (!initLidar(lidar)) return 1;
            return RunMappen(uart, lidar);

        case MenuKeuze::PICO_COMMUNICATIE:
            return RunPicoCommunicatie(uart);

        case MenuKeuze::RIJDEN_EN_MAPPEN:
            if (!initLidar(lidar)) return 1;
            return RunRijdenEnMappen(uart, lidar);

        case MenuKeuze::STOPPEN:
            std::cout << "Programma afgesloten.\n";
            return 0;
    }

    return 0;
}