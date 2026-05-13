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
#include <fstream>
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

// ── ObstakelRichting ─────────────────────────────────────────────────────────
//
//  Drie zones gebaseerd op fysieke maten:
//
//  KRITIEK (0–350mm)  : chassis kan niet meer draaien → stop, draai op plek
//  REMMEN  (350–600mm): rem af naar 40%, hard bijsturen
//  VEILIG  (600–900mm): vroeg beginnen bijsturen, vol gas
//
//  Returns:
//    0    = vrij
//   ±1    = VEILIG zijkant    (zachte bocht, vol gas)
//   ±2    = VEILIG midden     (zachte bocht, vol gas)
//   ±20   = REMMEN            (harde bocht, halve snelheid)
//   ±30   = KRITIEK           (stop + draai op plek)
static int ObstakelRichting(const Mapper& mapper, const Position& pos)
{
    static constexpr float DEG2RAD    = 3.14159265f / 180.0f;
    static constexpr float MM2M       = 0.001f;
    static constexpr float KRITIEK_MM =  350.0f;
    static constexpr float REMMEN_MM  =  600.0f;
    static constexpr float VEILIG_MM  =  900.0f;
    static constexpr float HOEK_DEG   =   20.0f;

    float thetaRad = pos.GetTheta() * DEG2RAD;
    float hoekRad  = HOEK_DEG * DEG2RAD;
    float robotX_m = pos.GetX() * MM2M;
    float robotY_m = pos.GetY() * MM2M;

    constexpr int STAPPEN = 18;

    bool kritiekVoor = false, kritiekLinks = false, kritiekRechts = false;
    bool remmenVoor  = false, remmenLinks  = false, remmenRechts  = false;
    bool veiligVoor  = false, veiligLinks  = false, veiligRechts  = false;

    for (int s = 1; s <= STAPPEN; ++s) {
        float r_mm = VEILIG_MM * static_cast<float>(s) / STAPPEN;
        float r_m  = r_mm * MM2M;

        const float offsets[3] = { 0.0f, -hoekRad, +hoekRad };
        for (int i = 0; i < 3; ++i) {
            float wx_m = robotX_m + r_m * std::cos(thetaRad + offsets[i]);
            float wy_m = robotY_m + r_m * std::sin(thetaRad + offsets[i]);
            int cx, cy;
            mapper.GetMap().WorldToCell(wx_m, wy_m, cx, cy);
            if (!mapper.GetMap().InBounds(cx, cy) || !mapper.GetMap().IsOccupied(cx, cy))
                continue;

            if (r_mm <= KRITIEK_MM) {
                if (i == 0) kritiekVoor   = true;
                if (i == 1) kritiekLinks  = true;
                if (i == 2) kritiekRechts = true;
            } else if (r_mm <= REMMEN_MM) {
                if (i == 0) remmenVoor    = true;
                if (i == 1) remmenLinks   = true;
                if (i == 2) remmenRechts  = true;
            } else {
                if (i == 0) veiligVoor    = true;
                if (i == 1) veiligLinks   = true;
                if (i == 2) veiligRechts  = true;
            }
        }
    }

    // Kritiek — stop en draai, kies vrije kant
    if (kritiekVoor || kritiekLinks || kritiekRechts) {
        if (kritiekRechts && !kritiekLinks) return -30;  // rechts vol → links draaien
        return +30;                                       // links/voor vol → rechts draaien
    }

    // Remmen — hard bijsturen, snelheid terug
    if (remmenVoor)   return +20;
    if (remmenRechts) return -20;
    if (remmenLinks)  return +20;

    // Veilig — vroeg zacht bijsturen
    if (veiligVoor)   return  +2;
    if (veiligRechts) return  -1;
    if (veiligLinks)  return  +1;

    return 0;
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
    // Obstakel
    int     obstRichting;        // 0=vrij, ±1=zijkant, 2=voor
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

    // ── Staat ──────────────────────────────────────────────────
    printf("╠══════════════════════════════════════════════════════════╣\n");
    const char* staatTekst = "";
    const char* staatKleur = "";
    switch (s.staat) {
        case 0: staatTekst = "VRIJ RIJDEN  (geen obstakel, geen pad — rechtdoor)";
                staatKleur = "\033[32m"; break;   // groen
        case 1: staatTekst = "NAVIGEERT    (pad actief, bijsturing via navigator)";
                staatKleur = "\033[36m"; break;   // cyaan
        case 2: staatTekst = "OBSTAKEL     (zachte bocht, obstakel gedetecteerd) ";
                staatKleur = "\033[33m"; break;   // geel
        case 3: staatTekst = "HERPLANNEN   (wacht op nieuw pad van A*)           ";
                staatKleur = "\033[35m"; break;   // magenta
    }
    printf("║  %s● %s\033[0m  ║\n", staatKleur, staatTekst);

    // Obstakelrichting visueel — drie zones
    const char* obstSymbool;
    switch (s.obstRichting) {
        case   0: obstSymbool = "  VRIJ    [voor: ---] [links: ---] [rechts: ---]               "; break;
        case  +1: obstSymbool = "\033[33m  VEILIG  [voor: ---] [links: ---] [rechts: 900mm] bocht L  \033[0m"; break;
        case  -1: obstSymbool = "\033[33m  VEILIG  [voor: ---] [links: 900mm] [rechts: ---] bocht R  \033[0m"; break;
        case  +2: obstSymbool = "\033[33m  VEILIG  [voor: 900mm] [links: ---] [rechts: ---] bocht L  \033[0m"; break;
        case +20: obstSymbool = "\033[33m  REMMEN  [voor/L: 600mm] rem+bocht L — 40% snelheid         \033[0m"; break;
        case -20: obstSymbool = "\033[33m  REMMEN  [rechts: 600mm] rem+bocht R — 40% snelheid         \033[0m"; break;
        case +30: obstSymbool = "\033[31m  KRITIEK [<350mm] STOP + draai L — chassis beschermd        \033[0m"; break;
        case -30: obstSymbool = "\033[31m  KRITIEK [<350mm] STOP + draai R — chassis beschermd        \033[0m"; break;
        default:  obstSymbool = "  [onbekend code]                                                  "; break;
    }
    printf("║  %s║\n", obstSymbool);

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
               << "afstand_doel_mm,hoekfout_deg,staat,obst_richting\n";
    }
    auto logStart = std::chrono::steady_clock::now();
    printf("Autonoom rijden + mappen gestart.\n");
    printf("Strategie: altijd vooruit (278 mm/s), zachte bochten bij obstakels.\n\n");
    usleep(1200000);   // LIDAR spin-up

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

        // ── 3a. Obstakelcheck: drie zones ────────────────────────
        int obstRichting = ObstakelRichting(mapper, pos);
        st.obstRichting  = obstRichting;

        if (obstRichting != 0) {
            float richting = (obstRichting > 0) ? -1.0f : +1.0f;
            int   absCode  = std::abs(obstRichting);

            if (absCode == 30) {
                // KRITIEK (<350mm): volledig stoppen, alleen draaien op de plek
                // Rijdt NIET vooruit — chassis kan niet meer draaien bij <250mm
                ka.SetCommand(0.0f, richting * 40.0f);
                nieuweScan = false;  // kaart niet bijwerken tijdens stilstaand draaien

            } else if (absCode == 20) {
                // REMMEN (350–600mm): rem af naar 40%, hard bijsturen
                ka.SetCommand(LIN_SPEED * 0.4f, richting * 20.0f);

            } else {
                // VEILIG (600–900mm): zacht bijsturen, vol gas
                ka.SetCommand(LIN_SPEED, richting * 8.0f);
            }
            st.staat = 2;

        // ── 3b. Navigator rijdt het pad ───────────────────────────
        } else if (heeftPad && !navigator.IsFinished()) {
            navigator.Update(pos);
            DriveCommand cmd = navigator.GetNextCommand(pos);
            ka.SetCommand(cmd.GetLinVelocity(), cmd.GetAngVelocity());
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
                ka.SetCommand(LIN_SPEED, 0.0f);
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
            csvLog << tijdMs                << ','
                   << st.posX              << ','
                   << st.posY              << ','
                   << st.theta             << ','
                   << st.speedLinks        << ','
                   << st.speedRechts       << ','
                   << st.imuYaw            << ','
                   << st.imuOmega          << ','
                   << st.cmdLin            << ','
                   << st.cmdAng            << ','
                   << st.dekking           << ','
                   << st.waypointHuidig    << ','
                   << st.waypointTotaal    << ','
                   << st.afstandTotDoel    << ','
                   << st.hoekFout          << ','
                   << st.staat             << ','
                   << st.obstRichting      << '\n';
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