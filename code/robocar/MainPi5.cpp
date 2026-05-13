#include "MainPi5.h"
#include "LIDAR.h"
#include "Mapper.h"
#include "Localisation.h"
#include "Pi5UARTHandler.h"
#include <csignal>
#include <chrono>
#include <thread>
#include <mutex>
#include <cstring>
#include <atomic>
#include <unistd.h>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <limits>

static std::atomic<bool> running{true};
static Pi5UARTHandler*   g_uart = nullptr;  // voor noodstop in signal handler

static void onSignal(int) {
    running = false;
    // Directe noodstop — stuurt STOP zonder te wachten op keepalive thread
    if (g_uart && g_uart->IsOpen()) {
        const char* stop = "STOP\nSTOP\nSTOP\n";
        // write() is async-signal-safe
        ::write(g_uart->GetFd(), stop, strlen(stop));
    }
}


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
                    if (actief && running)
                        uart.StuurCommand(lin, ang);
                    else if (!running) {
                        // Ctrl+C ontvangen: stuur stop en zet actief uit
                        uart.StuurStop();
                        actief = false;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
    Localisation loc(235.0f);
    // 13 m × 8 m testruimte, 5 cm per cel → 260 × 160 cellen
    Mapper mapper(260, 160, 0.05f);

    constexpr float DT      = 0.1f;
    constexpr long  LOOP_US = 100000;
    int scanCount = 0;

    std::cout << "\033[2J";
    std::cout << "Mapping gestart. Motor spin-up...\n";
    usleep(1200000);

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        uart.LeesData();
        SensorData sens = uart.GetSensorData();
        if (sens.geldig) {
            loc.Predict(sens.speedLinks, sens.speedRechts, DT);
            loc.UpdateIMU(sens.yawGraden, DT);
        }

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
        uart.LeesData();
        SensorData sens = uart.GetSensorData();

        if (sens.geldig)
            printf("ENC L=%7.2f mm/s  ENC R=%7.2f mm/s  IMU yaw=%7.2f deg  omega=%6.2f deg/s\n",
                   sens.speedLinks, sens.speedRechts,
                   sens.yawGraden,  sens.hoeksnelheid);
        else
            printf("Wacht op DATA van Pico...\n");

        std::cout << "─────────────────────────────────────────\n";
        usleep(100000);  // 10 Hz display
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
    constexpr float MM2M = 0.001f;
    float bestDist = 0.0f;
    // Startwaarden in meter
    float huidigX_m = huidig.GetX() * MM2M;
    float huidigY_m = huidig.GetY() * MM2M;
    float bestWx_m  = huidigX_m;
    float bestWy_m  = huidigY_m;

    int W = mapper.GetMap().GetWidth();
    int H = mapper.GetMap().GetHeight();

    for (int cy = 0; cy < H; ++cy) {
        for (int cx = 0; cx < W; ++cx) {
            if (!mapper.GetMap().IsFree(cx, cy)) continue;

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

            // CellToWorld geeft meter terug
            float wx_m, wy_m;
            mapper.GetMap().CellToWorld(cx, cy, wx_m, wy_m);

            float dx2 = wx_m - huidigX_m;
            float dy2 = wy_m - huidigY_m;
            float dist = std::sqrt(dx2*dx2 + dy2*dy2);

            if (dist > bestDist) {
                bestDist = dist;
                bestWx_m = wx_m;
                bestWy_m = wy_m;
            }
        }
    }

    // Geef terug in mm (consistent met de rest van het systeem)
    return Position(bestWx_m / MM2M, bestWy_m / MM2M, 0.0f);
}

// ════════════════════════════════════════════════════════════════
//  ScanAnalyse  —  resultaat van de 360° LIDAR-analyse
// ════════════════════════════════════════════════════════════════
struct ScanAnalyse {
    int   staat;          // 0=vrij  1=veilig  2=remmen  3=kritiek
    float uitwijkHoek;    // gewenste draaihoek t.o.v. robot (graden, + = links)
    float ruimteLinks;    // gemiddelde vrije ruimte linkerhelft (mm)
    float ruimteRechts;   // gemiddelde vrije ruimte rechterhelft (mm)
    float minVoor;        // minimale afstand in de voorwaartse sector (mm)
    float minLinks;       // minimale afstand linker sector (mm)
    float minRechts;      // minimale afstand rechter sector (mm)
};

// ── AnalyseerScan ────────────────────────────────────────────────────────────
//
//  Gebruikt de ruwe LIDAR-meting (360 waarden, index = LIDAR-hoek in graden).
//  robotTheta (graden) roteert de indices zodat sector 0 altijd "recht voor
//  de robot" is, ongeacht de rijrichting.
//
//  Sectorindeling (na rotatie, t.o.v. robot):
//    sector  0        =   0° – 10°   = recht voor
//    sector  4        =  40° – 50°   = voor-rechts
//    sector  9        =  90°–100°    = rechts
//    sector 18        = 180°–190°    = achter
//    sector 27        = 270°–280°    = links
//    sector 31        = 310°–320°    = voor-links
//
//  Fysieke constraints:
//    VEILIG_MM   = 900mm  — detectie begint hier, zacht bijsturen
//    REMMEN_MM   = 600mm  — rem naar 40%, hard bijsturen
//    KRITIEK_MM  = 350mm  — stop + draai op plek (chassis blokkeert <250mm)
//    CHASSIS_MM  = 300mm  — minimale zijruimte die de robot nodig heeft voor bocht
//
static ScanAnalyse AnalyseerScan(const float ranges[360], float robotTheta = 0.0f)
{
    static constexpr float KRITIEK_MM  = 350.0f;
    static constexpr float REMMEN_MM   = 600.0f;
    static constexpr float VEILIG_MM   = 900.0f;
    static constexpr float CHASSIS_MM  = 300.0f;
    static constexpr float MAX_RANGE   = 8000.0f;

    // ── Stap 1: roteer LIDAR-indices naar robot-frame ─────────
    // LIDAR index 0 = absolute 0°. De robot kijkt in richting robotTheta.
    // Offset = hoeveel graden we moeten opschuiven zodat robot-voorwaarts
    // op index 0 belandt.
    //   robotForwardLidarAngle: welk LIDAR-index overeenkomt met "recht voor"
    //   Als robotTheta = 90°, dan kijkt de robot naar 90° → LIDAR[90] is voor.
    //   We willen sector[0] = robot-voor, dus rotatie = +robotTheta.
    int rotOffset = static_cast<int>(std::fmod(robotTheta + 360.0f, 360.0f));

    // ── Stap 2: bouw sector-gemiddelden per 10° in robot-frame ─
    // sector[0]  = robot-voor (0°–10°)
    // sector[9]  = robot-rechts (90°–100°)
    // sector[27] = robot-links (270°–280°)
    float sector[36] = {};
    int   sectorN[36] = {};

    for (int a = 0; a < 360; ++a) {
        float r = ranges[a];
        if (r <= 0.0f || r > MAX_RANGE) r = MAX_RANGE;
        // Roteer: robot-frame hoek = (LIDAR-hoek - rotOffset + 360) % 360
        int robotAngle = (a - rotOffset + 360) % 360;
        int s = (robotAngle / 10) % 36;
        sector[s]  += r;
        sectorN[s] += 1;
    }
    for (int s = 0; s < 36; ++s)
        if (sectorN[s] > 0) sector[s] /= static_cast<float>(sectorN[s]);

    // ── Stap 3: sectorMin helper (wraparound) ─────────────────
    auto sectorMin = [&](int van, int tot) -> float {
        float m = MAX_RANGE;
        for (int s = van; s != (tot % 36); s = (s + 1) % 36)
            if (sector[s] < m) m = sector[s];
        return m;
    };

    // ── Stap 4: minimale afstand per zone (in robot-frame) ────
    //  Voorwaarts  = ±45° van robot-voor  → sector 0–4  en 32–35
    //  Voor-rechts = 45°–135°             → sector 5–13  (strikte rechterhelft)
    //  Voor-links  = 225°–315°            → sector 23–31 (strikte linkerhelft)
    //  Rechter zijkant (voor bocht check) = 45°–135°  → sector 5–13
    //  Linker  zijkant (voor bocht check) = 225°–315° → sector 23–31
    float minVoor        = sectorMin(32, 5);   // 320°–50°   robot-voor ±45°
    float minRechtsZijde = sectorMin(5,  14);  //  50°–140°  rechter zijkant
    float minLinksZijde  = sectorMin(23, 32);  // 230°–320°  linker zijkant
    // minLinks/minRechts exclusief het voor-gebied voor de display
    float minLinks       = sectorMin(23, 32);  // 230°–320°  links voor
    float minRechts      = sectorMin(5,  14);  //  50°–140°  rechts voor

    // ── Stap 5: gemiddelde ruimte links en rechts ─────────────
    // Linkerhelft = 180°–360° (sector 18–35), rechterhelft = 0°–180° (sector 0–17)
    float somLinks = 0.0f, somRechts = 0.0f;
    for (int s = 18; s < 36; ++s) somLinks  += sector[s];
    for (int s = 0;  s < 18; ++s) somRechts += sector[s];
    float ruimteLinks  = somLinks  / 18.0f;
    float ruimteRechts = somRechts / 18.0f;

    // ── Stap 6: vul resultaat ─────────────────────────────────
    ScanAnalyse res{};
    res.ruimteLinks  = ruimteLinks;
    res.ruimteRechts = ruimteRechts;
    res.minVoor      = minVoor;
    res.minLinks     = minLinks;
    res.minRechts    = minRechts;

    if (minVoor >= VEILIG_MM) {
        res.staat = 0;   // volledig vrij
        return res;
    }

    // ── Stap 7: kies uitwijkrichting ──────────────────────────
    // Links uitwijken vereist dat de linker zijkant > CHASSIS_MM heeft.
    bool linksVeilig  = (minLinksZijde  > CHASSIS_MM);
    bool rechtsVeilig = (minRechtsZijde > CHASSIS_MM);

    float scoreLinks  = linksVeilig  ? ruimteLinks  : 0.0f;
    float scoreRechts = rechtsVeilig ? ruimteRechts : 0.0f;

    // +1 = links uitwijken, -1 = rechts uitwijken
    res.uitwijkHoek = (scoreLinks >= scoreRechts) ? +1.0f : -1.0f;

    if (!linksVeilig && !rechtsVeilig)
        res.uitwijkHoek = (ruimteLinks >= ruimteRechts) ? +1.0f : -1.0f;

    // ── Stap 8: ernstklasse ───────────────────────────────────
    if (minVoor < KRITIEK_MM)
        res.staat = 3;
    else if (minVoor < REMMEN_MM)
        res.staat = 2;
    else
        res.staat = 1;

    return res;
}

// ════════════════════════════════════════════════════════════════
//  PrintStatus  —  live dashboard voor de autonome rijmodus
//
//  Gebruikt ANSI escape codes om het scherm op te refreshen zonder
//  te scrollen. Elke nieuweScan wordt het dashboard bijgewerkt.
//
//  Staat:
//    0 = VRIJ (geen obstakel, geen pad)
//    1 = NAVIGEERT (pad actief)
//    2 = OBSTAKEL (bocht aan het maken)
//    3 = HERPLAN (wacht op nieuw pad)
// ════════════════════════════════════════════════════════════════

struct RobotStatus {
    // Positie & oriëntatie
    float   posX, posY;          // mm
    float   theta;               // graden
    // Encoder
    float   speedLinks;          // mm/s
    float   speedRechts;         // mm/s
    // IMU
    float   imuYaw;              // graden
    float   imuOmega;            // graden/s
    // Aansturing
    float   cmdLin;              // mm/s
    float   cmdAng;              // deg/s
    // Navigatie
    bool    heeftPad;
    int     waypointHuidig;
    int     waypointTotaal;
    float   doelX, doelY;        // mm
    float   afstandTotDoel;      // mm
    float   hoekFout;            // graden
    // Obstakel / scan analyse
    int     obstRichting;        // 0=vrij 1=veilig 2=remmen 3=kritiek
    float   scanMinVoor;         // mm
    float   scanRuimteLinks;     // mm
    float   scanRuimteRechts;    // mm
    float   scanUitwijkHoek;     // +1=links -1=rechts
    // Kaart
    int     scanCount;
    int     dekking;             // %
    bool    robotCelVrij;
    // Staat
    int     staat;               // zie boven
    // EKF beschikbaarheid
    bool    encGeldig;
    bool    imuGeldig;
};

static void PrintStatus(const RobotStatus& s)
{
    // Cursor naar boven — refresh zonder scrollen
    // \033[H = cursor naar (0,0), \033[2J = clear screen
    printf("\033[H");

    // ── Titel ──────────────────────────────────────────────────
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║           ROBOT AUTONOOM — LIVE STATUS                  ║\n");
    printf("╠═══════════════════════════╦══════════════════════════════╣\n");

    // ── Kolom links: positie/oriëntatie | Kolom rechts: aansturing ──
    printf("║  LOKALISATIE              ║  AANSTURING                  ║\n");
    printf("║  X   : %8.0f mm        ║  Linear  : %7.1f mm/s      ║\n",
           s.posX, s.cmdLin);
    printf("║  Y   : %8.0f mm        ║  Angular : %7.1f deg/s     ║\n",
           s.posY, s.cmdAng);
    printf("║  θ   : %8.1f °          ║                              ║\n",
           s.theta);

    // ── Sensoren ───────────────────────────────────────────────
    printf("╠═══════════════════════════╬══════════════════════════════╣\n");
    printf("║  SENSOREN                 ║  NAVIGATIE                   ║\n");

    if (s.encGeldig) {
        printf("║  ENC L: %7.1f mm/s      ║  Pad     : %s             ║\n",
               s.speedLinks, s.heeftPad ? "JA  " : "NEE ");
        printf("║  ENC R: %7.1f mm/s      ║  Waypoint: %3d / %-3d        ║\n",
               s.speedRechts, s.waypointHuidig, s.waypointTotaal);
        printf("║  IMU θ: %7.1f °          ║  Doel    : (%6.0f,%6.0f)  ║\n",
               s.imuYaw, s.doelX, s.doelY);
        printf("║  ω    : %7.1f °/s        ║  Afstand : %7.0f mm       ║\n",
               s.imuOmega, s.afstandTotDoel);
    } else {
        printf("║  Sensoren: wacht op DATA  ║  Pad     : %s             ║\n",
               s.heeftPad ? "JA  " : "NEE ");
        printf("║  (Pico pusht elke 50ms)   ║  Waypoint: %3d / %-3d        ║\n",
               s.waypointHuidig, s.waypointTotaal);
        printf("║                           ║  Doel    : (%6.0f,%6.0f)  ║\n",
               s.doelX, s.doelY);
        printf("║                           ║  Afstand : %7.0f mm       ║\n",
               s.afstandTotDoel);
    }
    printf("║                           ║  HoekFout: %7.1f °         ║\n",
           s.hoekFout);

    // ── Kaart & obstakel ───────────────────────────────────────
    printf("╠═══════════════════════════╩══════════════════════════════╣\n");
    printf("║  KAART   Scans: %4d   Dekking: %3d%%   Cel: %s       ║\n",
           s.scanCount, s.dekking, s.robotCelVrij ? "VRIJ " : "FOUT!");

    // Voortgangsbalk voor kaartdekking (40 tekens breed)
    {
        int gevuld = s.dekking * 40 / 100;
        printf("║  [");
        for (int i = 0; i < 40; ++i) printf(i < gevuld ? "█" : "░");
        printf("]  %3d%%  ║\n", s.dekking);
    }

    // Obstakelstaat en 360° ruimte
    printf("╠══════════════════════════════════════════════════════════╣\n");
    const char* staatTekst = "";
    const char* staatKleur = "";
    switch (s.staat) {
        case 0: staatTekst = "VRIJ RIJDEN  (geen obstakel, geen pad — rechtdoor)";
                staatKleur = "\033[32m"; break;
        case 1: staatTekst = "NAVIGEERT    (pad actief, bijsturing via navigator)";
                staatKleur = "\033[36m"; break;
        case 2: staatTekst = "OBSTAKEL     (uitwijken op basis van 360° scan)    ";
                staatKleur = "\033[33m"; break;
        case 3: staatTekst = "HERPLANNEN   (wacht op nieuw pad van A*)           ";
                staatKleur = "\033[35m"; break;
    }
    printf("║  %s● %s\033[0m  ║\n", staatKleur, staatTekst);

    // 360° ruimte visueel
    const char* obstKleur =
        (s.obstRichting == 0) ? "\033[32m" :
        (s.obstRichting == 1) ? "\033[33m" :
        (s.obstRichting == 2) ? "\033[33m" : "\033[31m";
    const char* obstLabel =
        (s.obstRichting == 0) ? "VRIJ   " :
        (s.obstRichting == 1) ? "VEILIG " :
        (s.obstRichting == 2) ? "REMMEN " : "KRITIEK";
    const char* uitwijkLabel = (s.scanUitwijkHoek >= 0.0f) ? "← LINKS " : "RECHTS →";

    printf("║  %s%s\033[0m  Voor:%5.0fmm  Links:%5.0fmm  Rechts:%5.0fmm  %s  ║\n",
           obstKleur, obstLabel,
           s.scanMinVoor, s.scanRuimteLinks, s.scanRuimteRechts,
           (s.obstRichting > 0) ? uitwijkLabel : "        ");

    printf("╚══════════════════════════════════════════════════════════╝\n");
    fflush(stdout);
}

static int RunRijdenEnMappen(Pi5UARTHandler& uart, LIDAR& lidar)
{
    Localisation loc(235.0f);   // wheelBase in mm
    // 13 m × 8 m testruimte, 5 cm per cel → 260 × 160 cellen
    Mapper mapper(260, 160, 0.05f);

    constexpr float DT             = 0.1f;
    constexpr long  LOOP_US        = 100000;
    constexpr float LIN_SPEED      = 278.0f;
    constexpr int   HERPLAN_SCANS  =  15;

    int      scanCount         = 0;
    int      scansSindsHerplan = HERPLAN_SCANS;  // direct herplannen bij start

    // Pathfinding objecten
    PathPlanner planner(mapper.GetMap());
    Navigator   navigator;
    bool        heeftPad   = false;

    CommandKeepAlive ka(uart);
    ka.Start();

    // ── CSV log ───────────────────────────────────────────────
    // Stuur robot_log.csv naar Claude voor analyse na de rit.
    // Kolommen: tijd, positie, sensoren, commando, navigatie, staat
    std::ofstream csvLog("robot_log.csv");
    if (csvLog.is_open()) {
        csvLog << "tijd_ms,x_mm,y_mm,theta_deg,"
               << "encL_mms,encR_mms,yaw_imu_deg,omega_degs,"
               << "cmd_lin,cmd_ang,"
               << "dekking_pct,waypoint_huidig,waypoint_totaal,"
               << "afstand_doel_mm,hoekfout_deg,"
               << "staat,scan_staat,min_voor_mm,ruimte_links_mm,ruimte_rechts_mm\n";
    }
    auto logStart = std::chrono::steady_clock::now();

    // lastRanges buiten de loop zodat hij beschikbaar is in de scan-wacht sectie
    float lastRanges[360] = {};
    bool  heeftRanges     = false;

    printf("LIDAR spin-up (1.2s)...\n");
    usleep(1200000);

    // Wacht op eerste geldige scan VOORDAT de robot begint te rijden
    printf("Wacht op eerste LIDAR-scan...\n");
    for (int pogingen = 0; !heeftRanges && running; ++pogingen) {
        if (lidar.Update()) {
            float angles[360];
            for (int a = 0; a < 360; ++a) {
                lastRanges[a] = lidar.GetDistance(a).distance;
                angles[a]     = static_cast<float>(a);
            }
            heeftRanges = true;
            printf("Eerste scan ontvangen (poging %d). Start rijden.\n\n", pogingen + 1);
        } else {
            if (pogingen % 10 == 0 && pogingen > 0)
                printf("  Nog geen scan (%ds)...\n", pogingen / 10);
            usleep(100000);
            if (pogingen > 150) {
                printf("FOUT: LIDAR geeft geen data na 15s. Controleer /dev/ttyUSB0.\n");
                ka.Stop();
                ka.Shutdown();
                return 1;
            }
        }
    }

    ka.SetCommand(LIN_SPEED, 0.0f);   // begin rijden

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        // ── 1. Sensor-data lezen (non-blocking push) ─────────────
        uart.LeesData();
        SensorData sens = uart.GetSensorData();

        if (sens.geldig) {
            loc.Predict(sens.speedLinks, sens.speedRechts, DT);
            loc.UpdateIMU(sens.yawGraden, DT);
        }

        Position pos(loc.GetX(), loc.GetY(), loc.GetTheta());

        // ── 2. LIDAR → kaart ─────────────────────────────────────
        bool nieuweScan = false;

        if (lidar.Update()) {
            float angles[360];
            for (int a = 0; a < 360; ++a) {
                lastRanges[a] = lidar.GetDistance(a).distance;
                angles[a]     = static_cast<float>(a);
            }
            mapper.Update(lastRanges, angles, 360, pos);
            heeftRanges = true;
            ++scanCount;
            ++scansSindsHerplan;
            nieuweScan = true;
        }

        // Analyseer 360° scan — altijd, ook als geen nieuwe scan
        // Geef robotTheta mee zodat sector 0 altijd robot-voorwaarts is
        ScanAnalyse scan{};
        if (heeftRanges)
            scan = AnalyseerScan(lastRanges, loc.GetTheta());

        // ── 3. Autonoom rijden ────────────────────────────────────
        //
        //  Strategie: altijd vooruit rijden (linear=278).
        //  - Obstakel gedetecteerd → zachte bocht (angular ±8)
        //  - Pad beschikbaar       → navigator stuurt bij via proportionele gain
        //  - Geen pad / geen kaart → rechtdoor
        // ─────────────────────────────────────────────────────────

        // Status struct — elke iteratie vers gevuld voor PrintStatus
        RobotStatus st{};
        st.posX        = pos.GetX();
        st.posY        = pos.GetY();
        st.theta       = loc.GetTheta();
        st.speedLinks  = sens.geldig ? sens.speedLinks   : 0.0f;
        st.speedRechts = sens.geldig ? sens.speedRechts  : 0.0f;
        st.imuYaw      = sens.geldig ? sens.yawGraden    : 0.0f;
        st.imuOmega    = sens.geldig ? sens.hoeksnelheid : 0.0f;
        st.encGeldig   = sens.geldig;
        st.imuGeldig   = sens.geldig;
        st.scanCount   = scanCount;
        st.dekking     = mapper.GetCoverage();
        st.heeftPad    = heeftPad;

        {
            constexpr float MM2M = 0.001f;
            int cx = 0, cy = 0;
            mapper.GetMap().WorldToCell(pos.GetX() * MM2M, pos.GetY() * MM2M, cx, cy);
            st.robotCelVrij = mapper.GetMap().InBounds(cx, cy) &&
                              mapper.GetMap().IsFree(cx, cy);
        }

        // Navigatie-info voor de status
        if (heeftPad && !navigator.IsFinished()) {
            Position doel = navigator.GetCurrentTarget();
            st.doelX          = doel.GetX();
            st.doelY          = doel.GetY();
            st.waypointHuidig = navigator.GetPath().GetCurrentIndex() + 1;
            st.waypointTotaal = navigator.GetPath().GetSize();
            // Bereken afstand en hoekfout tot huidig waypoint
            float dx = doel.GetX() - pos.GetX();
            float dy = doel.GetY() - pos.GetY();
            st.afstandTotDoel = std::sqrt(dx*dx + dy*dy);
            float desired     = std::atan2(dy, dx) * (180.0f / static_cast<float>(M_PI));
            float err         = desired - loc.GetTheta();
            while (err >  180.0f) err -= 360.0f;
            while (err < -180.0f) err += 360.0f;
            st.hoekFout = err;
        }

        // ── 3a. Obstakelcheck op basis van 360° LIDAR-analyse ────
        //
        //  Prioriteitsregels:
        //    staat 3 (KRITIEK <350mm): altijd overschrijven, volledig stop + draai
        //    staat 2 (REMMEN 350–600mm): overschrijven, rem + bijsturen
        //    staat 1 (VEILIG 600–900mm): zachte bijsturing bovenop navigator
        //    staat 0 (VRIJ ≥900mm): navigator of rechtdoor
        //
        st.obstRichting     = scan.staat;
        st.scanMinVoor      = scan.minVoor;
        st.scanRuimteLinks  = scan.ruimteLinks;
        st.scanRuimteRechts = scan.ruimteRechts;
        st.scanUitwijkHoek  = scan.uitwijkHoek;

        if (heeftRanges && scan.staat >= 3) {
            // KRITIEK (<350mm): volledig stoppen, draai naar vrije kant
            float richting = scan.uitwijkHoek;  // +1 = links, -1 = rechts
            ka.SetCommand(0.0f, richting * 40.0f);
            st.staat = 2;

        } else if (heeftRanges && scan.staat == 2) {
            // REMMEN (350–600mm): rem naar 40%, hard bijsturen
            float richting = scan.uitwijkHoek;
            ka.SetCommand(LIN_SPEED * 0.4f, richting * 20.0f);
            st.staat = 2;

        } else if (heeftPad && !navigator.IsFinished()) {
            // ── 3b. Navigator rijdt het pad ───────────────────────────
            // Bij staat==1 (VEILIG): navigator-hoek + zachte obstakelcorrectie
            navigator.Update(pos);
            DriveCommand cmd = navigator.GetNextCommand(pos);
            float extraAng = 0.0f;
            if (heeftRanges && scan.staat == 1)
                extraAng = scan.uitwijkHoek * 8.0f;  // zachte bijsturing bovenop navigator
            ka.SetCommand(cmd.GetLinVelocity(), cmd.GetAngVelocity() + extraAng);
            st.staat = 1;

        } else {
            // ── 3c. Herplan als navigator klaar of geen pad ───────
            if (heeftPad && navigator.IsFinished()) {
                heeftPad          = false;
                st.heeftPad       = false;
                scansSindsHerplan = HERPLAN_SCANS;
            }

            if (nieuweScan && scansSindsHerplan >= HERPLAN_SCANS) {
                scansSindsHerplan = 0;
                st.staat          = 3;

                Position doel = KiesFrontierDoel(mapper, pos);

                if (doel.GetX() == pos.GetX() && doel.GetY() == pos.GetY()) {
                    ka.SetCommand(LIN_SPEED, 0.0f);
                    scansSindsHerplan = HERPLAN_SCANS;
                    st.staat = 0;
                } else {
                    st.doelX = doel.GetX();
                    st.doelY = doel.GetY();
                    Path pad = planner.PlanPath(pos, doel, mapper.GetMap());

                    if (!pad.IsEmpty()) {
                        navigator.SetPath(pad);
                        heeftPad          = true;
                        st.heeftPad       = true;
                        st.waypointTotaal = pad.GetSize();
                        st.staat          = 1;
                    } else {
                        ka.SetCommand(LIN_SPEED, 0.0f);
                        scansSindsHerplan = HERPLAN_SCANS;
                        st.staat = 0;
                    }
                }
            } else {
                // Geen pad of herplannen nodig
                // Bij staat==1 (VEILIG): al zacht bijsturen terwijl we wachten op een pad
                float extraAng = (heeftRanges && scan.staat == 1)
                                 ? scan.uitwijkHoek * 8.0f : 0.0f;
                ka.SetCommand(LIN_SPEED, extraAng);
                st.staat = 0;
            }
        }

        st.cmdLin = ka.GetLin();
        st.cmdAng = ka.GetAng();

        // ── 4. Dashboard printen (elke nieuwe scan) ───────────────
        if (nieuweScan)
            PrintStatus(st);

        // ── 5. CSV loggen (elke iteratie, ~100 Hz) ────────────────
        if (csvLog.is_open()) {
            long tijdMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - logStart).count();
            csvLog << tijdMs                 << ','
                   << st.posX               << ','
                   << st.posY               << ','
                   << st.theta              << ','
                   << st.speedLinks         << ','
                   << st.speedRechts        << ','
                   << st.imuYaw             << ','
                   << st.imuOmega           << ','
                   << st.cmdLin             << ','
                   << st.cmdAng             << ','
                   << st.dekking            << ','
                   << st.waypointHuidig     << ','
                   << st.waypointTotaal     << ','
                   << st.afstandTotDoel     << ','
                   << st.hoekFout           << ','
                   << st.staat              << ','
                   << st.obstRichting       << ','
                   << st.scanMinVoor        << ','
                   << st.scanRuimteLinks    << ','
                   << st.scanRuimteRechts   << '\n';
        }

        // ── 5. Loop timing ────────────────────────────────────────
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::steady_clock::now() - tStart).count();
        long rest = LOOP_US - elapsed;
        if (rest > 0) usleep(static_cast<useconds_t>(rest));
    }

    ka.Stop();
    ka.Shutdown();

    // Stuur stop meerdere keren — zeker dat de Pico het ontvangt
    for (int i = 0; i < 10; ++i) {
        uart.StuurStop();
        usleep(50000);  // 50ms tussen pogingen
    }
    printf("Noodstop gestuurd.\n");
    lidar.Disconnect();

    if (csvLog.is_open()) {
        csvLog.close();
        printf("Log opgeslagen: robot_log.csv\n");
    }

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
    std::cout << "║  3. Autonoom rijden + mappen         ║\n";
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

    g_uart = &uart;  // signal handler kan nu direct stoppen
    initUartHandler(uart);

    MenuKeuze keuze = VraagMenuKeuze();

    // Pico reboot is optioneel — geen ACK is geen probleem als Pico al draait
    if (!uart.RebootPico())
        std::cout << "Pico reboot overgeslagen (al actief).\n";
    else
        usleep(1500000);  // wacht op Pico boot alleen als reboot gelukt is

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