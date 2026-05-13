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
static Pi5UARTHandler*   g_uart = nullptr;

static void onSignal(int) {
    running = false;
    if (g_uart && g_uart->IsOpen()) {
        const char* stop = "STOP\nSTOP\nSTOP\n";
        ::write(g_uart->GetFd(), stop, strlen(stop));
    }
}


// ════════════════════════════════════════════════════════════════
//  CommandKeepAlive
// ════════════════════════════════════════════════════════════════

class CommandKeepAlive {
public:
    explicit CommandKeepAlive(Pi5UARTHandler& uart)
        : uart(uart), lin(0.0f), ang(0.0f), actief(false)
    {}

    void SetCommand(float linearMmS, float angularDegS) {
        std::lock_guard<std::mutex> lk(mtx);
        lin    = linearMmS;
        ang    = angularDegS;
        actief = true;
    }

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

    void Start() {
        threadActief = true;
        worker = std::thread([this]() {
            while (threadActief) {
                {
                    std::lock_guard<std::mutex> lk(mtx);
                    if (actief && running)
                        uart.StuurCommand(lin, ang);
                    else if (!running) {
                        uart.StuurStop();
                        actief = false;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }

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
    if (!uart.Open()) { std::cerr << "UART niet beschikbaar.\n"; return false; }
    return true;
}

static bool initLidar(LIDAR& lidar) {
    if (!lidar.Connect()) { std::cerr << "Kan niet verbinden met LIDAR.\n"; return false; }
    return true;
}


// ════════════════════════════════════════════════════════════════
//  Modus 1: Mappen
// ════════════════════════════════════════════════════════════════

static int RunMappen(Pi5UARTHandler& uart, LIDAR& lidar) {
    Localisation loc(235.0f);
    Mapper mapper(260, 160, 0.05f);
    constexpr float DT = 0.1f;
    constexpr long LOOP_US = 100000;
    int scanCount = 0;
    std::cout << "\033[2J";
    std::cout << "Mapping gestart. Motor spin-up...\n";
    usleep(1200000);
    while (running) {
        auto tStart = std::chrono::steady_clock::now();
        uart.LeesData();
        SensorData sens = uart.GetSensorData();
        if (sens.geldig) { loc.Predict(sens.speedLinks, sens.speedRechts, DT); loc.UpdateIMU(sens.yawGraden, DT); }
        if (!lidar.Update()) { usleep(200000); continue; }
        float ranges[360], angles[360];
        for (int a = 0; a < 360; ++a) { ranges[a] = lidar.GetDistance(a).distance; angles[a] = (float)a; }
        Position pos(loc.GetX(), loc.GetY(), loc.GetTheta());
        mapper.Update(ranges, angles, 360, pos);
        ++scanCount;
        printf("Scans: %4d  Dekking: %2d%%  Pos: (%.2f, %.2f)  theta: %.1fdeg\n",
               scanCount, mapper.GetCoverage(), loc.GetX(), loc.GetY(), loc.GetTheta());
        mapper.PrintMap(loc.GetX(), loc.GetY(), scanCount, mapper.GetCoverage());
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - tStart).count();
        long rest = LOOP_US - elapsed;
        if (rest > 0) usleep((useconds_t)rest);
    }
    std::cout << "\nStoppen...\n";
    lidar.Disconnect();
    if (mapper.SaveDebugMap("kaart.pgm")) printf("Kaart opgeslagen: kaart.pgm  (dekking: %d%%)\n", mapper.GetCoverage());
    return 0;
}


// ════════════════════════════════════════════════════════════════
//  Modus 2: Pico communicatie
// ════════════════════════════════════════════════════════════════

static void PrintPicoMenu(const CommandKeepAlive& ka) {
    std::cout << "\n╔══════════════════════════════════════╗\n║        PICO COMMUNICATIE             ║\n╠══════════════════════════════════════╣\n";
    if (ka.IsActief()) printf("║  Actief: lin=%-6.1f  ang=%-10.1f║\n", ka.GetLin(), ka.GetAng());
    else std::cout << "║  Actief: GESTOPT                     ║\n";
    std::cout << "╠══════════════════════════════════════╣\n║  1. DriveCommand sturen              ║\n║  2. Sensordata ontvangen (live)      ║\n║  3. Noodstop                         ║\n║  4. Terug naar hoofdmenu             ║\n╚══════════════════════════════════════╝\nKeuze: ";
}

static void RunStuurCommand(CommandKeepAlive& ka) {
    float lin = 0.0f, ang = 0.0f;
    std::cout << "Lineaire snelheid (mm/s): "; std::cin >> lin;
    std::cout << "Hoeksnelheid (deg/s):     "; std::cin >> ang;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    ka.SetCommand(lin, ang);
    printf("  Commando actief: lin=%.1f  ang=%.1f\n", lin, ang);
}

static void RunLeesLive(Pi5UARTHandler& uart) {
    std::cout << "Live sensordata (druk Enter om te stoppen)...\n\n";
    std::atomic<bool> stopLive{false};
    std::thread inputThread([&]() { std::string d; std::getline(std::cin, d); stopLive = true; });
    while (!stopLive) {
        uart.LeesData();
        SensorData sens = uart.GetSensorData();
        if (sens.geldig) printf("ENC L=%7.2f  ENC R=%7.2f  IMU yaw=%7.2f  omega=%6.2f\n", sens.speedLinks, sens.speedRechts, sens.yawGraden, sens.hoeksnelheid);
        else printf("Wacht op DATA van Pico...\n");
        std::cout << "─────────────────────────────────────────\n";
        usleep(100000);
    }
    if (inputThread.joinable()) inputThread.join();
}

static int RunPicoCommunicatie(Pi5UARTHandler& uart) {
    CommandKeepAlive ka(uart); ka.Start();
    while (true) {
        PrintPicoMenu(ka);
        std::string invoer; std::getline(std::cin, invoer);
        if      (invoer == "1") RunStuurCommand(ka);
        else if (invoer == "2") RunLeesLive(uart);
        else if (invoer == "3") { ka.Stop(); std::cout << "  Robot gestopt.\n"; }
        else if (invoer == "4") { ka.Stop(); ka.Shutdown(); break; }
        else std::cout << "Ongeldige keuze.\n";
    }
    return 0;
}


// ════════════════════════════════════════════════════════════════
//  Modus 3: Rijden + Mappen + autonoom pathfinding
// ════════════════════════════════════════════════════════════════

#include "PathPlanner.h"
#include "Navigator.h"

// ── Frontier-keuze: score op vrije LIDAR-ruimte (60%) + afstand (40%) ──
static Position KiesFrontierDoel(const Mapper& mapper, const Position& huidig,
                                  const float lidarRanges[360])
{
    constexpr float MM2M = 0.001f;
    float huidigX_m = huidig.GetX() * MM2M;
    float huidigY_m = huidig.GetY() * MM2M;
    int W = mapper.GetMap().GetWidth();
    int H = mapper.GetMap().GetHeight();
    float bestScore = -1.0f, bestWx_m = huidigX_m, bestWy_m = huidigY_m;

    for (int cy = 0; cy < H; ++cy) {
        for (int cx = 0; cx < W; ++cx) {
            if (!mapper.GetMap().IsFree(cx, cy)) continue;
            bool heeftOnbekendeBuur = false;
            const int dx[4]={-1,1,0,0}, dy[4]={0,0,-1,1};
            for (int d=0;d<4;++d) {
                int nx=cx+dx[d], ny=cy+dy[d];
                if (mapper.GetMap().InBounds(nx,ny) && mapper.GetMap().IsUnknown(nx,ny)) { heeftOnbekendeBuur=true; break; }
            }
            if (!heeftOnbekendeBuur) continue;
            float wx_m, wy_m;
            mapper.GetMap().CellToWorld(cx, cy, wx_m, wy_m);
            float ddx=wx_m-huidigX_m, ddy=wy_m-huidigY_m;
            float dist_m = std::sqrt(ddx*ddx+ddy*ddy);
            if (dist_m < 0.3f) continue;
            float hoek_rad = std::atan2(ddy,ddx) - (huidig.GetTheta()*(float)M_PI/180.0f);
            int lidarIdx = ((int)(hoek_rad*180.0f/(float)M_PI)+360)%360;
            float vrijRuimte = lidarRanges[lidarIdx];
            if (vrijRuimte<=0.0f||vrijRuimte>8000.0f) vrijRuimte=8000.0f;
            float score = 0.4f*std::min(dist_m,4.0f)/4.0f + 0.6f*std::min(vrijRuimte,2000.0f)/2000.0f;
            if (score > bestScore) { bestScore=score; bestWx_m=wx_m; bestWy_m=wy_m; }
        }
    }
    return Position(bestWx_m/MM2M, bestWy_m/MM2M, 0.0f);
}

static int TelFrontiers(const Mapper& mapper) {
    int W=mapper.GetMap().GetWidth(), H=mapper.GetMap().GetHeight(), count=0;
    const int dx[4]={-1,1,0,0}, dy[4]={0,0,-1,1};
    for (int cy=0;cy<H;++cy)
        for (int cx=0;cx<W;++cx) {
            if (!mapper.GetMap().IsFree(cx,cy)) continue;
            for (int d=0;d<4;++d)
                if (mapper.GetMap().InBounds(cx+dx[d],cy+dy[d]) && mapper.GetMap().IsUnknown(cx+dx[d],cy+dy[d])) { ++count; break; }
        }
    return count;
}


// ════════════════════════════════════════════════════════════════
//  ScanAnalyse
// ════════════════════════════════════════════════════════════════
struct ScanAnalyse {
    int   staat;
    float uitwijkHoek, ruimteLinks, ruimteRechts, minVoor, minLinks, minRechts;
    float minAchter;   // minimale ruimte achter de robot (sector 150°-210°)
};

static float g_vorigeUitwijk = 0.0f;

static ScanAnalyse AnalyseerScan(const float ranges[360])
{
    static constexpr float KRITIEK_MM = 350.0f;
    static constexpr float REMMEN_MM  = 500.0f;
    static constexpr float VEILIG_MM  = 700.0f;
    static constexpr float CHASSIS_MM = 280.0f;
    static constexpr float MAX_RANGE  = 8000.0f;
    static constexpr float HYSTERESIS = 200.0f;

    float sector[36];
    for (int s=0;s<36;++s) sector[s]=MAX_RANGE;
    for (int a=0;a<360;++a) {
        float r=ranges[a];
        if (r<=0.0f||r>MAX_RANGE) continue;
        int s=(a/10)%36;
        if (r<sector[s]) sector[s]=r;
    }
    auto sectorMin=[&](int van,int tot)->float {
        float m=MAX_RANGE;
        for (int s=van%36;s!=(tot%36);s=(s+1)%36) if (sector[s]<m) m=sector[s];
        return m;
    };

    float minVoorSmal=MAX_RANGE;
    { float m1=sectorMin(0,4),m2=sectorMin(33,36); minVoorSmal=(m1<m2)?m1:m2; }
    float minVoor=MAX_RANGE;
    { float m1=sectorMin(0,5),m2=sectorMin(31,36); minVoor=(m1<m2)?m1:m2; }

    float minRechtsZijde = sectorMin(5, 13);
    float minLinksZijde  = sectorMin(23, 31);
    float minRechts      = sectorMin(5, 18);
    float minLinks       = sectorMin(19, 35);
    float ruimteRechts   = sectorMin(1, 18);
    float ruimteLinks    = sectorMin(19, 36);
    // Achter-sector: index 15..21 = 150°..210°
    float minAchter      = sectorMin(15, 21);

    bool inGang = (minVoor>=REMMEN_MM)&&(minRechtsZijde<VEILIG_MM)&&(minLinksZijde<VEILIG_MM);

    ScanAnalyse res{};
    res.ruimteLinks=ruimteLinks; res.ruimteRechts=ruimteRechts;
    res.minVoor=minVoor; res.minLinks=minLinks; res.minRechts=minRechts;
    res.minAchter=minAchter;
    res.uitwijkHoek=g_vorigeUitwijk;

    if (minVoor>=VEILIG_MM||inGang) { res.staat=0; g_vorigeUitwijk=0.0f; return res; }

    bool rechtsVeilig=(minRechtsZijde>CHASSIS_MM), linksVeilig=(minLinksZijde>CHASSIS_MM);
    float scoreRechts=rechtsVeilig?ruimteRechts:0.0f, scoreLinks=linksVeilig?ruimteLinks:0.0f;
    if (g_vorigeUitwijk>0.0f) scoreRechts+=HYSTERESIS;
    if (g_vorigeUitwijk<0.0f) scoreLinks+=HYSTERESIS;
    float nieuweUitwijk=(!rechtsVeilig&&!linksVeilig)?((ruimteRechts>=ruimteLinks)?+1.0f:-1.0f):((scoreRechts>=scoreLinks)?+1.0f:-1.0f);
    res.uitwijkHoek=nieuweUitwijk; g_vorigeUitwijk=nieuweUitwijk;

    if      (minVoorSmal<KRITIEK_MM) res.staat=3;
    else if (minVoor    <REMMEN_MM)  res.staat=2;
    else                              res.staat=1;
    return res;
}


// ════════════════════════════════════════════════════════════════
//  PrintStatus
// ════════════════════════════════════════════════════════════════

struct RobotStatus {
    float posX,posY,theta;
    float speedLinks,speedRechts,imuYaw,imuOmega;
    float cmdLin,cmdAng;
    bool  heeftPad;
    int   waypointHuidig,waypointTotaal;
    float doelX,doelY,afstandTotDoel,hoekFout;
    int   obstRichting;
    float scanMinVoor,scanRuimteLinks,scanRuimteRechts,scanUitwijkHoek;
    int   scanCount,dekking;
    bool  robotCelVrij;
    int   staat;   // 0=vrij 1=navigeert 2=obstakel 3=herplan 4=terug_home 5=klaar
    bool  encGeldig,imuGeldig;
    int   aantalFrontiers;
    int   vastzitSec;  // seconden aaneengesloten in staat=2
};

static void PrintStatus(const RobotStatus& s) {
    printf("\033[H");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║           ROBOT AUTONOOM — LIVE STATUS                  ║\n");
    printf("╠═══════════════════════════╦══════════════════════════════╣\n");
    printf("║  LOKALISATIE              ║  AANSTURING                  ║\n");
    printf("║  X   : %8.0f mm        ║  Linear  : %7.1f mm/s      ║\n",s.posX,s.cmdLin);
    printf("║  Y   : %8.0f mm        ║  Angular : %7.1f deg/s     ║\n",s.posY,s.cmdAng);
    printf("║  θ   : %8.1f °          ║  Vastzit : %4ds             ║\n",s.theta,s.vastzitSec);
    printf("╠═══════════════════════════╬══════════════════════════════╣\n");
    printf("║  SENSOREN                 ║  NAVIGATIE                   ║\n");
    if (s.encGeldig) {
        printf("║  ENC L: %7.1f mm/s      ║  Pad     : %s             ║\n",s.speedLinks,s.heeftPad?"JA  ":"NEE ");
        printf("║  ENC R: %7.1f mm/s      ║  Waypoint: %3d / %-3d        ║\n",s.speedRechts,s.waypointHuidig,s.waypointTotaal);
        printf("║  IMU θ: %7.1f °          ║  Doel    : (%6.0f,%6.0f)  ║\n",s.imuYaw,s.doelX,s.doelY);
        printf("║  ω    : %7.1f °/s        ║  Afstand : %7.0f mm       ║\n",s.imuOmega,s.afstandTotDoel);
    } else {
        printf("║  Sensoren: wacht op DATA  ║  Pad     : %s             ║\n",s.heeftPad?"JA  ":"NEE ");
        printf("║  (Pico pusht elke 50ms)   ║  Waypoint: %3d / %-3d        ║\n",s.waypointHuidig,s.waypointTotaal);
        printf("║                           ║  Doel    : (%6.0f,%6.0f)  ║\n",s.doelX,s.doelY);
        printf("║                           ║  Afstand : %7.0f mm       ║\n",s.afstandTotDoel);
    }
    printf("║                           ║  HoekFout: %7.1f °         ║\n",s.hoekFout);
    printf("╠═══════════════════════════╩══════════════════════════════╣\n");
    printf("║  KAART  Scans:%4d  Dekking:%3d%%  Frontiers:%-4d  Cel:%s║\n",
           s.scanCount,s.dekking,s.aantalFrontiers,s.robotCelVrij?"VRIJ ":"FOUT!");
    { int g=s.dekking*40/100; printf("║  ["); for(int i=0;i<40;++i) printf(i<g?"█":"░"); printf("]  %3d%%  ║\n",s.dekking); }
    printf("╠══════════════════════════════════════════════════════════╣\n");
    const char* staatTekst="",*staatKleur="";
    switch(s.staat){
        case 0:staatTekst="VRIJ RIJDEN  (geen obstakel, geen pad — rechtdoor)";staatKleur="\033[32m";break;
        case 1:staatTekst="NAVIGEERT    (pad actief, bijsturing via navigator)";staatKleur="\033[36m";break;
        case 2:staatTekst="OBSTAKEL     (uitwijken op basis van 360° scan)    ";staatKleur="\033[33m";break;
        case 3:staatTekst="HERPLANNEN   (wacht op nieuw pad van A*)           ";staatKleur="\033[35m";break;
        case 4:staatTekst="TERUG HOME   (exploratie klaar, rijdt naar start)  ";staatKleur="\033[34m";break;
        case 5:staatTekst="KLAAR        (exploratie volledig, robot gestopt)   ";staatKleur="\033[32m";break;
    }
    printf("║  %s● %s\033[0m  ║\n",staatKleur,staatTekst);
    const char* obstKleur=(s.obstRichting==0)?"\033[32m":(s.obstRichting<=2)?"\033[33m":"\033[31m";
    const char* obstLabel=(s.obstRichting==0)?"VRIJ   ":(s.obstRichting==1)?"VEILIG ":(s.obstRichting==2)?"REMMEN ":"KRITIEK";
    const char* uitwijkLabel=(s.scanUitwijkHoek>=0.0f)?"← LINKS ":"RECHTS →";
    printf("║  %s%s\033[0m  Voor:%5.0fmm  Links:%5.0fmm  Rechts:%5.0fmm  %s  ║\n",
           obstKleur,obstLabel,s.scanMinVoor,s.scanRuimteLinks,s.scanRuimteRechts,
           (s.obstRichting>0)?uitwijkLabel:"        ");
    printf("╚══════════════════════════════════════════════════════════╝\n");
    fflush(stdout);
}


static int RunRijdenEnMappen(Pi5UARTHandler& uart, LIDAR& lidar)
{
    Localisation loc(235.0f);
    Mapper mapper(260, 160, 0.05f);

    constexpr float DT            = 0.1f;
    constexpr long  LOOP_US       = 100000;
    constexpr float LIN_SPEED     = 278.0f;
    constexpr int   HERPLAN_SCANS = 15;

    // ── Exploratie-eindcondities ──────────────────────────────────────
    // FRONTIER_DREMPEL: pas controleren als dekking >= MIN_DEKKING_PCT.
    // Bij lage dekking zijn er sowieso weinig FREE cellen en dus weinig
    // frontiers — dat is geen teken dat de kamer gescand is, maar dat
    // de robot nog nauwelijks heeft gereden.
    //
    // MIN_DEKKING_PCT: minimale kaartdekking voordat de eindconditie
    // mag triggeren. Zet dit op een realistisch getal voor jouw ruimte.
    // Voor een kamer van ~13×8m met 5cm cellen (260×160=41600 cellen)
    // betekent 30% dat ±12500 cellen bekend zijn — ruim genoeg om
    // zeker te zijn dat de robot echt heeft rondgereden.
    //
    // FRONTIER_DREMPEL: aantal resterende frontiers waaronder we
    // stoppen. 50 is ruim maar veilig — bij echte volledige exploratie
    // is dit getal heel klein (< 10).
    constexpr int   MIN_DEKKING_PCT   = 30;   // minimale dekking voor eindconditie
    constexpr int   FRONTIER_DREMPEL  = 50;   // frontiers waaronder = klaar
    constexpr int   MISLUKT_DREMPEL   = 8;    // achtereenvolgende A*-mislukkingen
    constexpr float HOME_DREMPEL_MM   = 300.0f;

    // ── Vastzit-timeout ───────────────────────────────────────────────
    // Als de robot meer dan VASTZIT_TIMEOUT_TICKS aaneengesloten in
    // ontwijkfase (staat=2) zit zonder dat zijn positie significant
    // verandert, geeft hij het op en rijdt hij naar huis.
    // 1 tick = 100ms → 400 ticks = 40 seconden.
    constexpr int   VASTZIT_TIMEOUT_TICKS = 400;
    constexpr float VASTZIT_BEWEG_MM      = 150.0f; // minimale beweging om "niet vast" te zijn

    int  scanCount         = 0;
    int  scansSindsHerplan = HERPLAN_SCANS;
    int  mislukteTeller    = 0;
    int  vastzitTicks      = 0;   // teller: ticks aaneengesloten in ontwijkfase
    float vastzitRefX      = 0.0f; // positie bij start vastzit-meting
    float vastzitRefY      = 0.0f;

    PathPlanner planner(mapper.GetMap());
    Navigator   navigator;
    bool        heeftPad = false;

    Position beginPunt(0.0f, 0.0f, 0.0f);
    bool     beginPuntVergrendeld = false;

    enum class ExploratieStaat { EXPLOREREN, TERUG_HOME, KLAAR };
    ExploratieStaat exploratieStaat = ExploratieStaat::EXPLOREREN;

    enum class OntwijkFase { NORMAAL, DRAAIEN, VRIJRIJDEN, ACHTERUIT };
    OntwijkFase ontwijkFase    = OntwijkFase::NORMAAL;
    float       doelHoek       = 0.0f;
    float       draaiRichting  = 0.0f;
    int         vrijrijTicks   = 0;
    int         achteruitTicks = 0;
    int         vastzitTeller  = 0;   // telt VRIJRIJDEN→KRITIEK cycli
    int         achteruitEscalatie = 0; // hoe vaak we al ACHTERUIT hebben geprobeerd
    constexpr int VRIJRIJ_TICKS   = 25;
    constexpr int ACHTERUIT_TICKS = 18;
    constexpr int VASTZIT_DREMPEL =  2;
    // Na ESCALATIE_DREMPEL keer ACHTERUIT zonder succes: langere achteruitrit
    constexpr int ESCALATIE_DREMPEL = 3;

    float ontsnapX = 1e9f, ontsnapY = 1e9f;
    constexpr float ONTSNAP_RADIUS = 700.0f;

    CommandKeepAlive ka(uart);
    ka.Start();

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

    float lastRanges[360] = {};
    bool  heeftRanges     = false;

    printf("LIDAR spin-up (1.2s)...\n");
    usleep(1200000);

    printf("Wacht op eerste LIDAR-scan...\n");
    for (int pogingen=0; !heeftRanges && running; ++pogingen) {
        if (lidar.Update()) {
            float angles[360];
            for (int a=0;a<360;++a) { lastRanges[a]=lidar.GetDistance(a).distance; angles[a]=(float)a; }
            heeftRanges=true;
            printf("Eerste scan ontvangen (poging %d). Start rijden.\n\n", pogingen+1);
        } else {
            if (pogingen%10==0&&pogingen>0) printf("  Nog geen scan (%ds)...\n", pogingen/10);
            usleep(100000);
            if (pogingen>150) { printf("FOUT: LIDAR geen data na 15s.\n"); ka.Stop(); ka.Shutdown(); return 1; }
        }
    }

    ka.SetCommand(LIN_SPEED, 0.0f);

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        // ── 1. Sensor-data lezen ─────────────────────────────────
        uart.LeesData();
        SensorData sens = uart.GetSensorData();
        if (sens.geldig) {
            loc.Predict(sens.speedLinks, sens.speedRechts, DT);
            loc.UpdateIMU(sens.yawGraden, DT);
            if (!beginPuntVergrendeld) {
                beginPunt = Position(loc.GetX(), loc.GetY(), loc.GetTheta());
                beginPuntVergrendeld = true;
            }
        }
        Position pos(loc.GetX(), loc.GetY(), loc.GetTheta());

        // ── 2. LIDAR → kaart ─────────────────────────────────────
        bool nieuweScan = false;
        if (lidar.Update()) {
            float angles[360];
            for (int a=0;a<360;++a) { lastRanges[a]=lidar.GetDistance(a).distance; angles[a]=(float)a; }
            mapper.Update(lastRanges, angles, 360, pos);
            heeftRanges=true; ++scanCount; ++scansSindsHerplan; nieuweScan=true;
        }

        ScanAnalyse scan{};
        if (heeftRanges) scan = AnalyseerScan(lastRanges);

        // ── 3. Vastzit-timeout bijhouden ─────────────────────────
        // Tel ticks aaneengesloten in ontwijkfase; reset als robot
        // voldoende beweegt of terugkeert naar NORMAAL.
        if (ontwijkFase != OntwijkFase::NORMAAL && exploratieStaat == ExploratieStaat::EXPLOREREN) {
            if (vastzitTicks == 0) {
                vastzitRefX = pos.GetX();
                vastzitRefY = pos.GetY();
            }
            ++vastzitTicks;

            float bewX = pos.GetX()-vastzitRefX, bewY = pos.GetY()-vastzitRefY;
            float beweging = std::sqrt(bewX*bewX+bewY*bewY);

            // Als de robot genoeg beweegt: reset de referentiepositie
            if (beweging > VASTZIT_BEWEG_MM) {
                vastzitRefX = pos.GetX();
                vastzitRefY = pos.GetY();
                vastzitTicks = 0;
            }

            // Timeout bereikt: geef het op voor dit gebied, ga naar huis
            if (vastzitTicks >= VASTZIT_TIMEOUT_TICKS) {
                printf("\n\033[31m⚠ Vastzit-timeout (%ds)! Opgegeven, rijdt naar huis.\033[0m\n",
                       VASTZIT_TIMEOUT_TICKS/10);
                exploratieStaat   = ExploratieStaat::TERUG_HOME;
                ontwijkFase       = OntwijkFase::NORMAAL;
                heeftPad          = false;
                vastzitTicks      = 0;
                vastzitTeller     = 0;
                achteruitEscalatie = 0;
                scansSindsHerplan = HERPLAN_SCANS;
                Path pad = planner.PlanPath(pos, beginPunt, mapper.GetMap());
                if (!pad.IsEmpty()) { navigator.SetPath(pad); heeftPad=true; }
            }
        } else {
            vastzitTicks = 0;
        }

        // ── 4. Status struct vullen ───────────────────────────────
        RobotStatus st{};
        st.posX=pos.GetX(); st.posY=pos.GetY(); st.theta=loc.GetTheta();
        st.speedLinks=sens.geldig?sens.speedLinks:0.0f;
        st.speedRechts=sens.geldig?sens.speedRechts:0.0f;
        st.imuYaw=sens.geldig?sens.yawGraden:0.0f;
        st.imuOmega=sens.geldig?sens.hoeksnelheid:0.0f;
        st.encGeldig=sens.geldig; st.imuGeldig=sens.geldig;
        st.scanCount=scanCount; st.dekking=mapper.GetCoverage(); st.heeftPad=heeftPad;
        st.aantalFrontiers=TelFrontiers(mapper);
        st.vastzitSec=vastzitTicks/10;
        {
            constexpr float MM2M=0.001f; int cx=0,cy=0;
            mapper.GetMap().WorldToCell(pos.GetX()*MM2M,pos.GetY()*MM2M,cx,cy);
            st.robotCelVrij=mapper.GetMap().InBounds(cx,cy)&&mapper.GetMap().IsFree(cx,cy);
        }
        if (heeftPad&&!navigator.IsFinished()) {
            Position doel=navigator.GetCurrentTarget();
            st.doelX=doel.GetX(); st.doelY=doel.GetY();
            st.waypointHuidig=navigator.GetPath().GetCurrentIndex()+1;
            st.waypointTotaal=navigator.GetPath().GetSize();
            float dx=doel.GetX()-pos.GetX(), dy=doel.GetY()-pos.GetY();
            st.afstandTotDoel=std::sqrt(dx*dx+dy*dy);
            float desired=std::atan2(dy,dx)*(180.0f/(float)M_PI);
            float err=desired-loc.GetTheta();
            while(err>180.0f)err-=360.0f; while(err<-180.0f)err+=360.0f;
            st.hoekFout=err;
        }
        st.obstRichting=scan.staat; st.scanMinVoor=scan.minVoor;
        st.scanRuimteLinks=scan.ruimteLinks; st.scanRuimteRechts=scan.ruimteRechts;
        st.scanUitwijkHoek=scan.uitwijkHoek;

        auto normDeg=[](float d)->float {
            while(d>180.0f)d-=360.0f; while(d<-180.0f)d+=360.0f; return d;
        };

        // ════════════════════════════════════════════════════════
        //  RIJLOGICA
        // ════════════════════════════════════════════════════════

        if (exploratieStaat == ExploratieStaat::KLAAR) {
            st.staat = 5;

        } else {
            // ── Obstakelontwijking (geldig in beide exploratietoestanden) ──
            if (ontwijkFase == OntwijkFase::DRAAIEN) {
                float huidig=loc.GetTheta(), fout=normDeg(doelHoek-huidig);
                if (std::fabs(fout)<8.0f) {
                    ontwijkFase=OntwijkFase::VRIJRIJDEN;
                    // Adaptief: langer vrijrijden bij escalatie (verder weg van muur)
                    int baseTicks = (scan.minVoor<600.0f) ? 10 : VRIJRIJ_TICKS;
                    vrijrijTicks = baseTicks + achteruitEscalatie * 8;
                    scansSindsHerplan=HERPLAN_SCANS;
                    ka.SetCommand(LIN_SPEED, 0.0f);
                } else if (heeftRanges&&scan.staat>=3&&
                           ((draaiRichting>0&&scan.minRechts<350.0f)||(draaiRichting<0&&scan.minLinks<350.0f))) {
                    draaiRichting=-draaiRichting;
                    doelHoek=normDeg(huidig-draaiRichting*90.0f);
                    ka.SetCommand(0.0f, draaiRichting*40.0f);
                } else {
                    ka.SetCommand(0.0f, draaiRichting*40.0f);
                }
                st.staat=2;

            } else if (ontwijkFase == OntwijkFase::VRIJRIJDEN) {
                if (heeftRanges&&scan.staat>=3) {
                    ++vastzitTeller;
                    if (vastzitTeller>=VASTZIT_DREMPEL) {
                        // Escalerende achteruitrit: elke keer langer
                        ++achteruitEscalatie;
                        int extraTicks = std::min(achteruitEscalatie * 8, 40);
                        achteruitTicks = ACHTERUIT_TICKS + extraTicks;
                        ontwijkFase=OntwijkFase::ACHTERUIT;
                        ka.SetCommand(-LIN_SPEED*0.6f, 0.0f);
                    } else {
                        draaiRichting=-draaiRichting;
                        doelHoek=normDeg(loc.GetTheta()-draaiRichting*90.0f);
                        ontwijkFase=OntwijkFase::DRAAIEN;
                        ka.SetCommand(0.0f, draaiRichting*40.0f);
                    }
                } else {
                    --vrijrijTicks;
                    if (vrijrijTicks<=0) {
                        vastzitTeller=0;      // alleen hier resetten
                        achteruitEscalatie=0; // succes: escalatie resetten
                        ontwijkFase=OntwijkFase::NORMAAL;
                        heeftPad=false;
                        scansSindsHerplan=HERPLAN_SCANS;
                        g_vorigeUitwijk=0.0f;
                    } else {
                        ka.SetCommand(LIN_SPEED, 0.0f);
                    }
                }
                st.staat=2;

            } else if (ontwijkFase == OntwijkFase::ACHTERUIT) {
                // ── Obstakelcheck ACHTER de robot ────────────────
                // Sector 150°-210° = wat achter de robot is.
                // Als die ook geblokkeerd is: stop met achteruitrijden en
                // probeer direct een nieuwe richting te draaien.
                bool achterGeblokkeerd = (heeftRanges && scan.minAchter < 300.0f);

                if (achterGeblokkeerd) {
                    // Kan niet achteruit: draai direct naar beste vrije richting
                    printf("  [ACHTERUIT geblokkeerd: achter=%.0fmm, kies nieuwe richting]\n", scan.minAchter);
                    float besteRuimte=0.0f, besteHoek=0.0f;
                    for (int s2=0;s2<36;++s2) {
                        float ruimte=8000.0f;
                        for (int a=s2*10;a<s2*10+10;++a) if (lastRanges[a]>0.0f&&lastRanges[a]<ruimte) ruimte=lastRanges[a];
                        if (ruimte>besteRuimte) { besteRuimte=ruimte; besteHoek=(float)(s2*10); }
                    }
                    if (besteHoek>180.0f) besteHoek-=360.0f;
                    doelHoek=normDeg(loc.GetTheta()+besteHoek);
                    draaiRichting=(besteHoek>=0.0f)?1.0f:-1.0f;
                    if (std::fabs(besteHoek)<5.0f) draaiRichting=1.0f;
                    ontwijkFase=OntwijkFase::DRAAIEN;
                    ka.SetCommand(0.0f, draaiRichting*40.0f);
                    ontsnapX=loc.GetX(); ontsnapY=loc.GetY();

                } else {
                    --achteruitTicks;
                    if (achteruitTicks<=0) {
                        // Kies richting met meeste vrije LIDAR-ruimte
                        float besteRuimte=0.0f, besteHoek=0.0f;
                        for (int s2=0;s2<36;++s2) {
                            float ruimte=8000.0f;
                            for (int a=s2*10;a<s2*10+10;++a) if (lastRanges[a]>0.0f&&lastRanges[a]<ruimte) ruimte=lastRanges[a];
                            if (ruimte>besteRuimte) { besteRuimte=ruimte; besteHoek=(float)(s2*10); }
                        }
                        if (besteHoek>180.0f) besteHoek-=360.0f;
                        doelHoek=normDeg(loc.GetTheta()+besteHoek);
                        draaiRichting=(besteHoek>=0.0f)?1.0f:-1.0f;
                        if (std::fabs(besteHoek)<5.0f) draaiRichting=1.0f;
                        ontwijkFase=OntwijkFase::DRAAIEN;
                        ka.SetCommand(0.0f, draaiRichting*40.0f);
                        ontsnapX=loc.GetX(); ontsnapY=loc.GetY();
                    } else {
                        ka.SetCommand(-LIN_SPEED*0.6f, 0.0f);
                    }
                }
                st.staat=2;

            } else {
                // ── OntwijkFase::NORMAAL ─────────────────────────
                if (exploratieStaat == ExploratieStaat::TERUG_HOME) {
                    st.staat=4;
                    float dx=pos.GetX()-beginPunt.GetX(), dy=pos.GetY()-beginPunt.GetY();
                    float afstandHome=std::sqrt(dx*dx+dy*dy);

                    if (afstandHome<HOME_DREMPEL_MM) {
                        ka.Stop();
                        exploratieStaat=ExploratieStaat::KLAAR;
                        printf("\n\033[32m✓ Thuis aangekomen! Exploratie klaar.\033[0m\n");
                        if (mapper.SaveDebugMap("kaart.pgm")) printf("Kaart opgeslagen: kaart.pgm\n");
                    } else if (!heeftPad||navigator.IsFinished()) {
                        heeftPad=false;
                        Path pad=planner.PlanPath(pos, beginPunt, mapper.GetMap());
                        if (!pad.IsEmpty()) { navigator.SetPath(pad); heeftPad=true; }
                        else {
                            float hoekNaarHome=std::atan2(-dy,-dx)*(180.0f/(float)M_PI);
                            float hoekFout2=normDeg(hoekNaarHome-loc.GetTheta());
                            ka.SetCommand(LIN_SPEED*0.6f, hoekFout2*0.5f);
                        }
                    }
                    if (heeftPad&&!navigator.IsFinished()) {
                        navigator.Update(pos);
                        DriveCommand cmd=navigator.GetNextCommand(pos);
                        float extraAng=(heeftRanges&&scan.staat==1)?scan.uitwijkHoek*8.0f:0.0f;
                        ka.SetCommand(cmd.GetLinVelocity(), cmd.GetAngVelocity()+extraAng);
                    }
                    // Obstakel tijdens terugrit → ontwijkfase
                    if (heeftRanges&&scan.staat>=3) {
                        ontsnapX=loc.GetX(); ontsnapY=loc.GetY();
                        draaiRichting=scan.uitwijkHoek;
                        doelHoek=normDeg(loc.GetTheta()-draaiRichting*90.0f);
                        ontwijkFase=OntwijkFase::DRAAIEN;
                        heeftPad=false;
                        ka.SetCommand(0.0f, draaiRichting*40.0f);
                        st.staat=2;
                    }

                } else {
                    // ── EXPLOREREN ────────────────────────────────
                    if (heeftRanges&&scan.staat>=3) {
                        ontsnapX=loc.GetX(); ontsnapY=loc.GetY();
                        draaiRichting=scan.uitwijkHoek;
                        doelHoek=normDeg(loc.GetTheta()-draaiRichting*90.0f);
                        ontwijkFase=OntwijkFase::DRAAIEN;
                        heeftPad=false;
                        ka.SetCommand(0.0f, draaiRichting*40.0f);
                        st.staat=2;

                    } else if (heeftRanges&&scan.staat==2) {
                        ka.SetCommand(LIN_SPEED*0.35f, scan.uitwijkHoek*22.0f);
                        st.staat=2;

                    } else if (heeftPad&&!navigator.IsFinished()) {
                        navigator.Update(pos);
                        DriveCommand cmd=navigator.GetNextCommand(pos);
                        float extraAng=(heeftRanges&&scan.staat==1)?scan.uitwijkHoek*8.0f:0.0f;
                        ka.SetCommand(cmd.GetLinVelocity(), cmd.GetAngVelocity()+extraAng);
                        st.staat=1;

                    } else {
                        if (heeftPad&&navigator.IsFinished()) {
                            heeftPad=false; st.heeftPad=false; scansSindsHerplan=HERPLAN_SCANS;
                        }
                        if (nieuweScan&&scansSindsHerplan>=HERPLAN_SCANS) {
                            scansSindsHerplan=0; st.staat=3;

                            int aantalFrontiers=TelFrontiers(mapper);
                            int huidigeDekking=mapper.GetCoverage();
                            // Eindconditie: alleen triggeren als dekking hoog genoeg is.
                            // Bij lage dekking heeft de robot gewoon nog niet genoeg
                            // van de kamer gezien om frontiers te beoordelen.
                            bool dekkingVoldoende = (huidigeDekking >= MIN_DEKKING_PCT);
                            if (dekkingVoldoende &&
                                (aantalFrontiers<=FRONTIER_DREMPEL||mislukteTeller>=MISLUKT_DREMPEL)) {
                                printf("\n\033[34m→ Exploratie klaar (dekking=%d%%, frontiers=%d, mislukt=%d). Rijdt terug.\033[0m\n",
                                       huidigeDekking, aantalFrontiers, mislukteTeller);
                                exploratieStaat=ExploratieStaat::TERUG_HOME;
                                heeftPad=false; mislukteTeller=0; scansSindsHerplan=HERPLAN_SCANS;
                                Path pad=planner.PlanPath(pos, beginPunt, mapper.GetMap());
                                if (!pad.IsEmpty()) { navigator.SetPath(pad); heeftPad=true; }
                            } else {
                                Position doel=KiesFrontierDoel(mapper, pos, lastRanges);
                                float dx=doel.GetX()-ontsnapX, dy=doel.GetY()-ontsnapY;
                                bool teVlakBijOntsnap=(dx*dx+dy*dy)<(ONTSNAP_RADIUS*ONTSNAP_RADIUS);
                                if (teVlakBijOntsnap&&ontsnapX<1e8f) {
                                    Position nepPos(ontsnapX,ontsnapY,pos.GetTheta());
                                    Position alt=KiesFrontierDoel(mapper,nepPos,lastRanges);
                                    float d1sq=dx*dx+dy*dy, dx2=alt.GetX()-ontsnapX, dy2=alt.GetY()-ontsnapY;
                                    if (dx2*dx2+dy2*dy2>d1sq) doel=alt;
                                    else ontsnapX=ontsnapY=1e9f;
                                }
                                if (doel.GetX()==pos.GetX()&&doel.GetY()==pos.GetY()) {
                                    ++mislukteTeller; ka.SetCommand(LIN_SPEED,0.0f);
                                    scansSindsHerplan=HERPLAN_SCANS; st.staat=0;
                                } else {
                                    st.doelX=doel.GetX(); st.doelY=doel.GetY();
                                    Path pad=planner.PlanPath(pos,doel,mapper.GetMap());
                                    if (!pad.IsEmpty()) {
                                        navigator.SetPath(pad); heeftPad=true; st.heeftPad=true;
                                        st.waypointTotaal=pad.GetSize(); st.staat=1; mislukteTeller=0;
                                    } else {
                                        ++mislukteTeller; ka.SetCommand(LIN_SPEED,0.0f);
                                        scansSindsHerplan=HERPLAN_SCANS; st.staat=0;
                                    }
                                }
                            }
                        } else {
                            float extraAng=(heeftRanges&&scan.staat==1)?scan.uitwijkHoek*8.0f:0.0f;
                            ka.SetCommand(LIN_SPEED,extraAng); st.staat=0;
                        }
                    }
                }
            }
        }

        st.cmdLin=ka.GetLin(); st.cmdAng=ka.GetAng();
        if (nieuweScan) PrintStatus(st);

        if (csvLog.is_open()) {
            long tijdMs=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-logStart).count();
            csvLog<<tijdMs<<','<<st.posX<<','<<st.posY<<','<<st.theta<<','
                  <<st.speedLinks<<','<<st.speedRechts<<','<<st.imuYaw<<','<<st.imuOmega<<','
                  <<st.cmdLin<<','<<st.cmdAng<<','<<st.dekking<<','<<st.waypointHuidig<<','<<st.waypointTotaal<<','
                  <<st.afstandTotDoel<<','<<st.hoekFout<<','<<st.staat<<','<<st.obstRichting<<','
                  <<st.scanMinVoor<<','<<st.scanRuimteLinks<<','<<st.scanRuimteRechts<<'\n';
        }

        if (exploratieStaat==ExploratieStaat::KLAAR) running=false;

        auto elapsed=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-tStart).count();
        long rest=LOOP_US-elapsed;
        if (rest>0) usleep((useconds_t)rest);
    }

    ka.Stop(); ka.Shutdown();
    for (int i=0;i<10;++i) { uart.StuurStop(); usleep(50000); }
    printf("Robot gestopt.\n");
    lidar.Disconnect();
    if (csvLog.is_open()) { csvLog.close(); printf("Log opgeslagen: robot_log.csv\n"); }
    if (mapper.SaveDebugMap("kaart.pgm")) printf("Kaart opgeslagen: kaart.pgm  (dekking: %d%%)\n", mapper.GetCoverage());
    return 0;
}


// ════════════════════════════════════════════════════════════════
//  main
// ════════════════════════════════════════════════════════════════

enum class MenuKeuze { MAPPEN, PICO_COMMUNICATIE, RIJDEN_EN_MAPPEN, STOPPEN };

static void PrintMenu() {
    std::cout<<"\n╔══════════════════════════════════════╗\n║        ROBOT CONTROLLER  Pi5         ║\n╠══════════════════════════════════════╣\n║  1. Mappen                           ║\n║  2. Pico communiceren                ║\n║  3. Autonoom rijden + mappen         ║\n║  4. Stoppen                          ║\n╚══════════════════════════════════════╝\nKeuze: ";
}

static MenuKeuze VraagMenuKeuze() {
    while (true) {
        PrintMenu();
        std::string invoer; std::getline(std::cin, invoer);
        if (invoer=="1") return MenuKeuze::MAPPEN;
        if (invoer=="2") return MenuKeuze::PICO_COMMUNICATIE;
        if (invoer=="3") return MenuKeuze::RIJDEN_EN_MAPPEN;
        if (invoer=="4") return MenuKeuze::STOPPEN;
        std::cout<<"Ongeldige keuze.\n";
    }
}

int main() {
    signal(SIGINT, onSignal);
    Pi5UARTHandler uart("/dev/ttyAMA10");
    LIDAR lidar("/dev/ttyUSB0", 460800);
    g_uart=&uart;
    initUartHandler(uart);
    MenuKeuze keuze=VraagMenuKeuze();
    if (!uart.RebootPico()) std::cout<<"Pico reboot overgeslagen (al actief).\n";
    else usleep(1500000);
    switch (keuze) {
        case MenuKeuze::MAPPEN:            if (!initLidar(lidar)) return 1; return RunMappen(uart, lidar);
        case MenuKeuze::PICO_COMMUNICATIE: return RunPicoCommunicatie(uart);
        case MenuKeuze::RIJDEN_EN_MAPPEN:  if (!initLidar(lidar)) return 1; return RunRijdenEnMappen(uart, lidar);
        case MenuKeuze::STOPPEN:           std::cout<<"Programma afgesloten.\n"; return 0;
    }
    return 0;
}