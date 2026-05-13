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
//  LIDAR-indices zijn robot-relatief: index 0 = recht voor, 90 = rechts,
//  270 = links. Geen rotatie-correctie nodig.
//
//  TEKENCONVENTIE angular (Pico-hardware):
//    ang > 0  → rechts draaien   (rechterwiel langzamer)
//    ang < 0  → links  draaien   (linkerwiel langzamer)
//  Geverifieerd: handmatig 0,−40 = links; 0,+40 = rechts.
//
//  uitwijkHoek:
//    +1.0 → uitwijken naar rechts (ang = +getal naar Pico)
//    −1.0 → uitwijken naar links  (ang = −getal naar Pico)
//
//  Nauwe gang logica:
//    Als voor VRIJ is maar links+rechts dicht → gewoon doorrijden (gang).
//    REMMEN alleen als er ook ruimte is om te draaien.
//    KRITIEK alleen als voor echt geblokkeerd is.
//
static float  g_vorigeUitwijk = 0.0f;  // hysteresis geheugen (+1=rechts, -1=links)

static ScanAnalyse AnalyseerScan(const float ranges[360])
{
    static constexpr float KRITIEK_MM  = 350.0f;
    static constexpr float REMMEN_MM   = 500.0f;
    static constexpr float VEILIG_MM   = 700.0f;  // was 900 — te vroeg in nauwe ruimtes
    static constexpr float CHASSIS_MM  = 280.0f;
    static constexpr float MAX_RANGE   = 8000.0f;
    static constexpr float HYSTERESIS  = 200.0f;

    // ── Stap 1: sector-minima per 10° ─────────────────────────
    float sector[36];
    for (int s = 0; s < 36; ++s) sector[s] = MAX_RANGE;

    for (int a = 0; a < 360; ++a) {
        float r = ranges[a];
        if (r <= 0.0f || r > MAX_RANGE) continue;
        int s = (a / 10) % 36;
        if (r < sector[s]) sector[s] = r;
    }

    auto sectorMin = [&](int van, int tot) -> float {
        float m = MAX_RANGE;
        for (int s = van % 36; s != (tot % 36); s = (s + 1) % 36)
            if (sector[s] < m) m = sector[s];
        return m;
    };

    // ── Stap 2: zones berekenen ───────────────────────────────
    //  Smalle voorzone (±30°): voor directe botsingsdetectie
    //  Brede voorzone  (±50°): voor VEILIG-detectie
    //  Zijkanten: voor bocht-veiligheid en gang-detectie
    float minVoorSmal = MAX_RANGE;   // ±30° — harde blokkade
    {
        float m1 = sectorMin(0,  4);   //   0°– 40°
        float m2 = sectorMin(33, 36);  // 330°–360°
        minVoorSmal = (m1 < m2) ? m1 : m2;
    }
    float minVoor = MAX_RANGE;        // ±50° — zachte blokkade
    {
        float m1 = sectorMin(0,  5);   //   0°– 50°
        float m2 = sectorMin(31, 36);  // 310°–360°
        minVoor = (m1 < m2) ? m1 : m2;
    }

    float minRechtsZijde = sectorMin(5,  13);  //  50°–130°
    float minLinksZijde  = sectorMin(23, 31);  // 230°–310°

    // Voor display
    float minRechts = sectorMin(5,  18);
    float minLinks  = sectorMin(19, 35);

    float ruimteRechts = sectorMin(1,  18);
    float ruimteLinks  = sectorMin(19, 36);

    // ── Stap 3: gang-detectie ─────────────────────────────────
    //  In een nauwe gang zijn links+rechts dicht maar voor is vrij.
    //  Dan NIET remmen of uitwijken — gewoon doorrijden.
    //  Gang-conditie: voor > REMMEN_MM maar beide zijkanten < VEILIG_MM
    bool inGang = (minVoor >= REMMEN_MM) &&
                  (minRechtsZijde < VEILIG_MM) &&
                  (minLinksZijde  < VEILIG_MM);

    // ── Stap 4: basisresultaat ────────────────────────────────
    ScanAnalyse res{};
    res.ruimteLinks  = ruimteLinks;
    res.ruimteRechts = ruimteRechts;
    res.minVoor      = minVoor;
    res.minLinks     = minLinks;
    res.minRechts    = minRechts;
    res.uitwijkHoek  = g_vorigeUitwijk;

    // Vrij of in gang: geen obstakelactie nodig
    if (minVoor >= VEILIG_MM || inGang) {
        res.staat = 0;
        g_vorigeUitwijk = 0.0f;
        return res;
    }

    // ── Stap 5: uitwijkrichting met hysteresis ────────────────
    bool rechtsVeilig = (minRechtsZijde > CHASSIS_MM);
    bool linksVeilig  = (minLinksZijde  > CHASSIS_MM);

    float scoreRechts = rechtsVeilig ? ruimteRechts : 0.0f;
    float scoreLinks  = linksVeilig  ? ruimteLinks  : 0.0f;

    if (g_vorigeUitwijk > 0.0f) scoreRechts += HYSTERESIS;
    if (g_vorigeUitwijk < 0.0f) scoreLinks  += HYSTERESIS;

    float nieuweUitwijk;
    if (!rechtsVeilig && !linksVeilig) {
        nieuweUitwijk = (ruimteRechts >= ruimteLinks) ? +1.0f : -1.0f;
    } else {
        nieuweUitwijk = (scoreRechts >= scoreLinks) ? +1.0f : -1.0f;
    }

    res.uitwijkHoek = nieuweUitwijk;
    g_vorigeUitwijk = nieuweUitwijk;

    // ── Stap 6: ernstklasse ───────────────────────────────────
    //  Gebruik minVoorSmal voor KRITIEK zodat schuine muren niet
    //  vals triggeren terwijl de rijlijn nog vrij is.
    if      (minVoorSmal < KRITIEK_MM) res.staat = 3;
    else if (minVoor     < REMMEN_MM)  res.staat = 2;
    else                               res.staat = 1;

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

    // ── Ontwijkfase state-machine ─────────────────────────────
    //   NORMAAL    = gewoon rijden/navigeren
    //   DRAAIEN    = op plek draaien naar doelhoek (lin=0, ang=±40)
    //   VRIJRIJDEN = na de draai een stukje vooruit (timer)
    //   ACHTERUIT  = vastzit-escape: achteruit rijden + nieuwe draai
    enum class OntwijkFase { NORMAAL, DRAAIEN, VRIJRIJDEN, ACHTERUIT };
    OntwijkFase ontwijkFase    = OntwijkFase::NORMAAL;
    float       doelHoek       = 0.0f;   // gewenste theta na draai (graden, −180…180]
    float       draaiRichting  = 0.0f;   // +1=rechts (ang>0), −1=links (ang<0)
    int         vrijrijTicks   = 0;      // afteller voor VRIJRIJDEN
    int         achteruitTicks = 0;      // afteller voor ACHTERUIT
    int         vastzitTeller  = 0;      // hoe vaak VRIJRIJDEN direct KRITIEK raakte
    constexpr int VRIJRIJ_TICKS   = 25;  // ~2.5s vooruit  ≈ 70cm
    constexpr int ACHTERUIT_TICKS = 18;  // ~1.8s achteruit ≈ 50cm
    constexpr int VASTZIT_DREMPEL =  2;  // na 2× vastlopen → achteruit

    // Ontsnappositie-geheugen: na een ontwijkmanoeuvre slaan we op waar
    // we vandaan kwamen. De frontier-keuze slaat dit gebied over zodat
    // de pathplanner de robot niet meteen terug dezelfde hoek in stuurt.
    float ontsnapX = 1e9f, ontsnapY = 1e9f;   // mm, wereld-coördinaten
    constexpr float ONTSNAP_RADIUS = 700.0f;   // mm — geen frontier binnen deze cirkel

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
        ScanAnalyse scan{};
        if (heeftRanges)
            scan = AnalyseerScan(lastRanges);

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

        // ── 3a. Obstakelcheck + ontwijkfase state-machine ────────
        //
        //  Tekenconventie samengevat:
        //    uitwijkHoek = +1 → rechts uitwijken → ang = +40 → theta daalt
        //    uitwijkHoek = -1 → links  uitwijken → ang = -40 → theta stijgt
        //    doelHoek = theta + draaiRichting * (-90°)
        //             = theta - 90° bij rechts draaien
        //             = theta + 90° bij links  draaien
        //
        //  OntwijkFase::DRAAIEN
        //    Draai op plek naar doelHoek. Stop als hoekfout < 8°.
        //    Als tijdens draaien KRITIEK in nieuwe richting: herbereken doelhoek.
        //    → klaar: ga naar VRIJRIJDEN
        //
        //  OntwijkFase::VRIJRIJDEN
        //    Rijd VRIJRIJ_TICKS × 100ms rechtdoor.
        //    Als scan.staat >= 3 tijdens vrijrijden: terug naar DRAAIEN.
        //    → klaar: herplan pad, terug naar NORMAAL
        //
        st.obstRichting     = scan.staat;
        st.scanMinVoor      = scan.minVoor;
        st.scanRuimteLinks  = scan.ruimteLinks;
        st.scanRuimteRechts = scan.ruimteRechts;
        st.scanUitwijkHoek  = scan.uitwijkHoek;

        // hulpfunctie: normaliseer graden naar (-180, 180]
        auto normDeg = [](float d) -> float {
            while (d >  180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            return d;
        };

        if (ontwijkFase == OntwijkFase::DRAAIEN) {
            // ── Fase DRAAIEN: draai op plek naar doelHoek ────────
            float huidig = loc.GetTheta();
            float fout   = normDeg(doelHoek - huidig);

            if (std::fabs(fout) < 8.0f) {
                // Doelhoek bereikt → ga vrijrijden
                ontwijkFase  = OntwijkFase::VRIJRIJDEN;
                vrijrijTicks = VRIJRIJ_TICKS;
                scansSindsHerplan = HERPLAN_SCANS;  // direct herplannen na vrijrijden
                ka.SetCommand(LIN_SPEED, 0.0f);
            } else if (heeftRanges && scan.staat >= 3 &&
                       // Blokkade in de richting waar we naartoe draaien?
                       // draaiRichting=+1 (rechts): rechter voorkant (minRechts) kan geblokkeerd raken
                       // draaiRichting=-1 (links):  linker  voorkant (minLinks)  kan geblokkeerd raken
                       ((draaiRichting > 0 && scan.minRechts < 350.0f) ||
                        (draaiRichting < 0 && scan.minLinks  < 350.0f))) {
                // Blokkade in draairichting: keer om
                draaiRichting = -draaiRichting;
                doelHoek = normDeg(huidig - draaiRichting * 90.0f);
                ka.SetCommand(0.0f, draaiRichting * 40.0f);
            } else {
                // Blijf draaien in de vastgezette richting
                ka.SetCommand(0.0f, draaiRichting * 40.0f);
            }
            st.staat = 2;

        } else if (ontwijkFase == OntwijkFase::VRIJRIJDEN) {
            // ── Fase VRIJRIJDEN: rijd een stuk vooruit ───────────
            if (heeftRanges && scan.staat >= 3) {
                // Direct KRITIEK na de draai → vastzitteller ophogen
                ++vastzitTeller;
                if (vastzitTeller >= VASTZIT_DREMPEL) {
                    // Meerdere keren vastgelopen → achteruit uit de hoek
                    vastzitTeller  = 0;
                    achteruitTicks = ACHTERUIT_TICKS;
                    ontwijkFase    = OntwijkFase::ACHTERUIT;
                    ka.SetCommand(-LIN_SPEED, 0.0f);
                } else {
                    // Nog één kans: draai de andere kant op
                    draaiRichting = -draaiRichting;
                    doelHoek      = normDeg(loc.GetTheta() - draaiRichting * 90.0f);
                    ontwijkFase   = OntwijkFase::DRAAIEN;
                    ka.SetCommand(0.0f, draaiRichting * 40.0f);
                }
            } else {
                --vrijrijTicks;
                if (vrijrijTicks <= 0) {
                    // Klaar — ontsnapX/Y blijft voor frontier-keuze
                    vastzitTeller     = 0;
                    ontwijkFase       = OntwijkFase::NORMAAL;
                    heeftPad          = false;
                    scansSindsHerplan = HERPLAN_SCANS;
                    g_vorigeUitwijk   = 0.0f;
                } else {
                    ka.SetCommand(LIN_SPEED, 0.0f);
                }
            }
            st.staat = 2;

        } else if (ontwijkFase == OntwijkFase::ACHTERUIT) {
            // ── Fase ACHTERUIT: rijd achteruit uit de hoek ───────
            // Geen obstakelcheck achteren — de kaart weet waar muren zijn
            // en de LIDAR achter de robot zit in sector 170°–190°.
            // We rijden blind achteruit voor een vaste tijd; kort genoeg
            // dat de kans op achterwaartse botsing klein is (~50cm).
            --achteruitTicks;
            if (achteruitTicks <= 0) {
                // Achteruit klaar → draai 180° en herplan
                draaiRichting = (scan.uitwijkHoek != 0.0f)
                                ? scan.uitwijkHoek : 1.0f;  // kies een kant
                doelHoek      = normDeg(loc.GetTheta() - draaiRichting * 150.0f);
                ontwijkFase   = OntwijkFase::DRAAIEN;
                ka.SetCommand(0.0f, draaiRichting * 40.0f);
                // Reset ontsnappositie — we zijn nu op een nieuwe plek
                ontsnapX = loc.GetX();
                ontsnapY = loc.GetY();
            } else {
                ka.SetCommand(-LIN_SPEED, 0.0f);
            }
            st.staat = 2;

        } else {
            // ── Fase NORMAAL ──────────────────────────────────────
            if (heeftRanges && scan.staat >= 3) {
                // KRITIEK: sla huidige positie op als ontsnappositie,
                // bereken doelhoek en begin draaien
                ontsnapX      = loc.GetX();
                ontsnapY      = loc.GetY();
                draaiRichting = scan.uitwijkHoek;           // +1=rechts (ang>0), -1=links (ang<0)
                doelHoek      = normDeg(loc.GetTheta() - draaiRichting * 90.0f);
                ontwijkFase   = OntwijkFase::DRAAIEN;
                heeftPad      = false;   // huidig pad vervalt
                ka.SetCommand(0.0f, draaiRichting * 40.0f);
                st.staat = 2;

            } else if (heeftRanges && scan.staat == 2) {
                // REMMEN: langzaam + bijsturen, blijf in NORMAAL
                ka.SetCommand(LIN_SPEED * 0.35f, scan.uitwijkHoek * 22.0f);
                st.staat = 2;

            // ── 3b. Navigator rijdt het pad ───────────────────────
            } else if (heeftPad && !navigator.IsFinished()) {
                navigator.Update(pos);
                DriveCommand cmd = navigator.GetNextCommand(pos);
                // VEILIG (staat==1): zachte obstakelcorrectie bovenop navigator
                float extraAng = (heeftRanges && scan.staat == 1)
                                 ? scan.uitwijkHoek * 8.0f : 0.0f;
                ka.SetCommand(cmd.GetLinVelocity(), cmd.GetAngVelocity() + extraAng);
                st.staat = 1;

            } else {
                // ── 3c. Herplan als navigator klaar of geen pad ───
                if (heeftPad && navigator.IsFinished()) {
                    heeftPad          = false;
                    st.heeftPad       = false;
                    scansSindsHerplan = HERPLAN_SCANS;
                }

                if (nieuweScan && scansSindsHerplan >= HERPLAN_SCANS) {
                    scansSindsHerplan = 0;
                    st.staat          = 3;

                    Position doel = KiesFrontierDoel(mapper, pos);

                    // Na een ontwijkmanoeuvre: sla frontiers over die te dicht
                    // bij de ontsnappositie liggen — anders stuurt de pathplanner
                    // de robot meteen terug dezelfde hoek in.
                    float dx = doel.GetX() - ontsnapX;
                    float dy = doel.GetY() - ontsnapY;
                    bool teVlakBijOntsnap = (dx*dx + dy*dy) < (ONTSNAP_RADIUS * ONTSNAP_RADIUS);

                    if (teVlakBijOntsnap && ontsnapX < 1e8f) {
                        // Kies een alternatief doel dat verder weg ligt
                        // Simpel: verdubbel de zoekafstand door de map te vragen
                        // om de op-één-na-dichtstbijzijnde frontier te kiezen.
                        // We doen dit door de ontsnappositie tijdelijk als
                        // huidige positie door te geven (zoek vandaar weg).
                        Position nepPos(ontsnapX, ontsnapY, pos.GetTheta());
                        Position alternatief = KiesFrontierDoel(mapper, nepPos);
                        // Pak het verste van de twee
                        float d1sq = dx*dx + dy*dy;
                        float dx2 = alternatief.GetX() - ontsnapX;
                        float dy2 = alternatief.GetY() - ontsnapY;
                        float d2sq = dx2*dx2 + dy2*dy2;
                        if (d2sq > d1sq) doel = alternatief;
                        // Als ook alternatief te dicht is: vergeet ontsnappositie
                        // zodat de robot niet voor altijd geblokkeerd blijft.
                        else ontsnapX = ontsnapY = 1e9f;
                    }

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
                    // Wacht op herplan: rechtdoor + zachte correctie bij VEILIG
                    float extraAng = (heeftRanges && scan.staat == 1)
                                     ? scan.uitwijkHoek * 8.0f : 0.0f;
                    ka.SetCommand(LIN_SPEED, extraAng);
                    st.staat = 0;
                }
            }
        } // einde OntwijkFase::NORMAAL

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