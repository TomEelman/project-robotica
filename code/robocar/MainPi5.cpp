#include "MainPi5.h"
#include "LIDAR.h"
#include "Mapper.h"
#include "Localisation.h"
#include "PathPlanner.h"
#include "Navigator.h"
#include "ExplorationPlanner.h"
#include "ScanMatcher.h"
#include <csignal>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <unistd.h>
#include <termios.h>

std::atomic<bool> running{true};
static Pi5UARTHandler* g_uart = nullptr;

static void onSignal(int) {
    running = false;
    if (g_uart && g_uart->IsOpen()) {
        const char* stop = "STOP\nSTOP\nSTOP\n";
        ::write(g_uart->GetFd(), stop, strlen(stop));
    }
}

CommandKeepAlive::CommandKeepAlive(Pi5UARTHandler& uart)
    : uart(uart), 
    lin(0.0f), 
    ang(0.0f), 
    actief(false) {}

void CommandKeepAlive::SetCommand(float linearMmS, float angularDegS) {
    std::lock_guard<std::mutex> lk(mtx);
    lin = linearMmS; ang = angularDegS; actief = true;
}

void CommandKeepAlive::Stop() {
    { std::lock_guard<std::mutex> lk(mtx); lin = ang = 0.0f; actief = false; }
    uart.StuurStop();
}

void CommandKeepAlive::Start() {
    threadActief = true;
    worker = std::thread([this]() {
        while (threadActief) {
            { std::lock_guard<std::mutex> lk(mtx);
                if (actief && running) uart.StuurCommand(lin, ang);
                else if (!running) { uart.StuurStop(); actief = false; }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
}

void CommandKeepAlive::Shutdown() {
    threadActief = false;
    if (worker.joinable()) worker.join();
}

bool CommandKeepAlive::IsActief() const { return actief.load(); }
float CommandKeepAlive::GetLin() const { return lin; }
float CommandKeepAlive::GetAng() const { return ang; }

// ─────────────────────────────────────────────────────────────────
// Enums
// ─────────────────────────────────────────────────────────────────

// HoofdModus: MUUR_VOLGEN verwijderd — robot start direct in FRONTIER.
enum class HoofdModus { FRONTIER, TERUG_HOME, KLAAR };
enum class OntwijkFase { NORMAAL, DRAAIEN, VRIJRIJDEN, ACHTERUIT };

// ─────────────────────────────────────────────────────────────────
// Helper declaraties
// ─────────────────────────────────────────────────────────────────
static bool HandleObstacleAvoidance(const ScanAnalysis& scan, Localisation& loc,
    OntwijkFase& ontwijkFase, float& doelHoek, float& draaiRichting,
    int& vrijrijTicks, int& achteruitTicks, int& vastzitTeller, int& achteruitEscalatie,
    float& ontsnapX, float& ontsnapY, CommandKeepAlive& ka, float LIN_SPEED, int& scansSindsHerplan);

static void HandleFrontierMode(Navigator& navigator, Mapper& mapper, Position pos, bool nieuweScan,
    int& scansSindsHerplan, int& mislukteTeller, HoofdModus& hoofdModus, bool& heeftPad,
    float ontsnapX, float ontsnapY, CommandKeepAlive& ka, PathPlanner& planner,
    std::vector<BlacklistItem>& frontierBlacklist, int HERPLAN_SCANS, int MIN_DEKKING_PCT,
    int MISLUKT_DREMPEL, float ONTSNAP_RADIUS, const float* lastRanges,
    float minFront);

static void HandleReturnToHome(Navigator& navigator, Mapper& mapper, Position pos, Position beginPunt,
    bool& heeftPad, CommandKeepAlive& ka, PathPlanner& planner, float HOME_DREMPEL_MM, HoofdModus& hoofdModus);

// ─────────────────────────────────────────────────────────────────
// Helper implementaties
// ─────────────────────────────────────────────────────────────────
static bool HandleObstacleAvoidance(const ScanAnalysis& scan, Localisation& loc,
    OntwijkFase& ontwijkFase, float& doelHoek, float& draaiRichting,
    int& vrijrijTicks, int& achteruitTicks, int& vastzitTeller, int& achteruitEscalatie,
    float& ontsnapX, float& ontsnapY, CommandKeepAlive& ka, float LIN_SPEED, int& scansSindsHerplan) {

    if (ontwijkFase == OntwijkFase::DRAAIEN) {
        float fout = NormDeg(doelHoek - loc.GetTheta());
        if (std::fabs(fout) < 8.0f) {
            ontwijkFase = OntwijkFase::VRIJRIJDEN;
            vrijrijTicks = (scan.minFront < 600.0f ? 10 : 25) + achteruitEscalatie * 8;
            scansSindsHerplan = 15;
            ka.SetCommand(LIN_SPEED, 0.0f);
        } else if (scan.state >= 3) {
            draaiRichting = -draaiRichting;
            doelHoek = NormDeg(loc.GetTheta() - draaiRichting * 90.0f);
            ka.SetCommand(0.0f, draaiRichting * 40.0f);
        } else {
            ka.SetCommand(0.0f, draaiRichting * 40.0f);
        }
    } else if (ontwijkFase == OntwijkFase::VRIJRIJDEN) {
        if (scan.state >= 3) {
            if (++vastzitTeller >= 2) {
                ++achteruitEscalatie;
                achteruitTicks = 18 + std::min(achteruitEscalatie * 8, 40);
                ontwijkFase = OntwijkFase::ACHTERUIT;
                ka.SetCommand(-LIN_SPEED * 0.6f, 0.0f);
            } else {
                draaiRichting = -draaiRichting;
                doelHoek = NormDeg(loc.GetTheta() - draaiRichting * 90.0f);
                ontwijkFase = OntwijkFase::DRAAIEN;
                ka.SetCommand(0.0f, draaiRichting * 40.0f);
            }
        } else if (--vrijrijTicks <= 0) {
            vastzitTeller = achteruitEscalatie = 0;
            ontwijkFase = OntwijkFase::NORMAAL;
            scansSindsHerplan = 15;
        } else {
            ka.SetCommand(LIN_SPEED, 0.0f);
        }
    } else if (ontwijkFase == OntwijkFase::ACHTERUIT) {
        if (scan.minRear < 300.0f || --achteruitTicks <= 0) {
            float bh = 0.0f;
            doelHoek = NormDeg(loc.GetTheta() + bh);
            draaiRichting = (bh >= 0.0f ? 1.0f : -1.0f);
            ontwijkFase = OntwijkFase::DRAAIEN;
            ka.SetCommand(0.0f, draaiRichting * 40.0f);
            ontsnapX = loc.GetX(); ontsnapY = loc.GetY();
        } else {
            ka.SetCommand(-LIN_SPEED * 0.6f, 0.0f);
        }
    }
    return true;
}

// HandleWallFollowing is verwijderd — robot navigeert puur via frontier/Wavefront.
// ComputeWallCommand in Navigator blijft beschikbaar voor eventueel later gebruik.

static void HandleFrontierMode(Navigator& navigator, Mapper& mapper, Position pos, bool nieuweScan,
    int& scansSindsHerplan, int& mislukteTeller, HoofdModus& hoofdModus, bool& heeftPad,
    float ontsnapX, float ontsnapY, CommandKeepAlive& ka, PathPlanner& planner,
    std::vector<BlacklistItem>& frontierBlacklist, int HERPLAN_SCANS, int MIN_DEKKING_PCT,
    int MISLUKT_DREMPEL, float ONTSNAP_RADIUS, const float* lastRanges,
    float minFront) {

    // Muurvolger-check verwijderd: robot schakelt niet meer automatisch
    // naar MUUR_VOLGEN zodra hij een muur naast zich detecteert.
    // Obstakels worden afgehandeld door HandleObstacleAvoidance in de hoofdlus.

    if (heeftPad && !navigator.IsFinished()) {
        navigator.Update(pos);
        DriveCommand cmd = navigator.GetNextCommand(pos, minFront);
        ka.SetCommand(cmd.GetLinVelocity(), cmd.GetAngVelocity());

        // Replan als de Navigator klem zat en een herstelactie uitvoerde
        if (navigator.IsBlocked()) {
            printf("[MAIN] Navigator blocked -> replan\n");
            Position bt = navigator.GetCurrentTarget();
            VoegToeAanBlacklist(frontierBlacklist, bt.GetX(), bt.GetY());
            navigator.ResetBlock();
            heeftPad = false;
            scansSindsHerplan = HERPLAN_SCANS;
        }

        if (navigator.IsBlocked()) {
            printf("[MAIN] Navigator blocked -> wacht op stabiele lokalisatie\n");
            Position bt = navigator.GetCurrentTarget();
            VoegToeAanBlacklist(frontierBlacklist, bt.GetX(), bt.GetY());
            navigator.ResetBlock();
            heeftPad = false;
            scansSindsHerplan = 0;  // forceer herplan pas bij volgende scan-cyclus
            mislukteTeller = 0;     // reset zodat hij niet meteen naar TERUG_HOME gaat
        }

    } else {
        if (heeftPad && navigator.IsFinished()) {
            heeftPad = false;
            scansSindsHerplan = HERPLAN_SCANS;
        }
        if (nieuweScan) {
            scansSindsHerplan = 0;
            int dekking = mapper.GetCoverage();
            printf("[MAIN] FRONTIER dekking=%d%% mislukt=%d\n", dekking, mislukteTeller);

            if (dekking >= MIN_DEKKING_PCT || mislukteTeller >= MISLUKT_DREMPEL) {
                hoofdModus = HoofdModus::TERUG_HOME;
                heeftPad = false;
                mislukteTeller = 0;
                printf("[MAIN] Klaar (%d%%) -> TERUG_HOME\n", dekking);
                Path pad = planner.PlanPath(pos, Position(0.0f, 0.0f, 0.0f), mapper.GetMap());
                if (!pad.IsEmpty()) { navigator.SetPath(pad); mapper.SetWaypoints(pad); heeftPad = true; }
            } else {
                TickBlacklist(frontierBlacklist);
                Position doel = KiesFrontierDoel(mapper, pos, lastRanges, frontierBlacklist);
                if (doel.GetX() == pos.GetX() && doel.GetY() == pos.GetY()) {
                    ++mislukteTeller;
                    ka.SetCommand(200.0f, 0.0f);
                } else {
                    Path pad = planner.PlanPath(pos, doel, mapper.GetMap());
                    if (!pad.IsEmpty()) {
                        navigator.SetPath(pad);
                        mapper.SetWaypoints(pad);
                        heeftPad = true;
                        mislukteTeller = 0;
                    } else {
                        ++mislukteTeller;
                        VoegToeAanBlacklist(frontierBlacklist, doel.GetX(), doel.GetY());
                        ka.SetCommand(200.0f, 0.0f);
                    }
                }
            }
        }
        // Geen nieuwe scan → ka herhaalt het laatste commando vanzelf
    }
}

static void HandleReturnToHome(Navigator& navigator, Mapper& mapper, Position pos, Position beginPunt,
    bool& heeftPad, CommandKeepAlive& ka, PathPlanner& planner, float HOME_DREMPEL_MM, HoofdModus& hoofdModus) {

    float dx = pos.GetX() - beginPunt.GetX();
    float dy = pos.GetY() - beginPunt.GetY();
    if (std::hypot(dx, dy) < HOME_DREMPEL_MM) {
        ka.Stop();
        mapper.SaveDebugMap("kaart.pgm");
        printf("[MAIN] Thuis aangekomen!\n");
        hoofdModus = HoofdModus::KLAAR;
        running = false;
        return;
    } else {
        if (!heeftPad || navigator.IsFinished()) {
            heeftPad = false;
            Path pad = planner.PlanPath(pos, beginPunt, mapper.GetMap());
            if (!pad.IsEmpty()) {
                navigator.SetPath(pad);
                mapper.SetWaypoints(pad);
                heeftPad = true;
            } else {
                float hoek = static_cast<float>(std::atan2(-dy, -dx)) * (180.0f / M_PI);
                ka.SetCommand(200.0f, NormDeg(hoek - pos.GetTheta()) * 0.5f);
            }
        }
        if (heeftPad && !navigator.IsFinished()) {
            navigator.Update(pos);
            DriveCommand cmd = navigator.GetNextCommand(pos);
            ka.SetCommand(cmd.GetLinVelocity(), cmd.GetAngVelocity());
        }
    }
}

// ─────────────────────────────────────────────────────────────────
// RunRijdenEnMappen
// ─────────────────────────────────────────────────────────────────
static int RunRijdenEnMappen(Pi5UARTHandler& uart, LIDAR& lidar) {
    Localisation loc(219.0f);
    Mapper mapper(500, 500, 0.03f);

    constexpr float DT          = 0.1f;
    constexpr long  LOOP_US     = 100000;
    constexpr float LIN_SPEED   = 278.0f;
    constexpr int   HERPLAN_SCANS   = 15;
    constexpr int   MIN_DEKKING_PCT = 30;
    constexpr int   MISLUKT_DREMPEL = 5;
    constexpr float HOME_DREMPEL_MM = 300.0f;
    constexpr float ONTSNAP_RADIUS  = 700.0f;
    constexpr float scanDuurSec     = 0.1f;

    // Robot start direct in FRONTIER — geen muuromloop meer
    HoofdModus hoofdModus = HoofdModus::FRONTIER;

    int scanCount         = 0;
    int scansSindsHerplan = HERPLAN_SCANS;
    int mislukteTeller    = 0;

    constexpr int   VASTZIT_TIMEOUT  = 400;
    constexpr float VASTZIT_BEWEG_MM = 150.0f;

    int   vastzitTicks = 0;
    float vastzitRefX  = 0.0f, vastzitRefY = 0.0f;

    float imuOffset  = 0.0f;
    bool  imuGenulld = false;

    float huidigeImuYaw = 0.0f;
    float omegaDegS     = 0.0f;

    std::vector<BlacklistItem> frontierBlacklist;
    PathPlanner planner(mapper.GetMap(), true);
    Navigator   navigator;
    bool        heeftPad = false;

    Position beginPunt(0.0f, 0.0f, 0.0f);
    bool     beginPuntVergrendeld = false;

    OntwijkFase ontwijkFase      = OntwijkFase::NORMAAL;
    float       doelHoek         = 0.0f;
    float       draaiRichting    = 0.0f;
    int         vrijrijTicks     = 0, achteruitTicks = 0,
                vastzitTeller_loc = 0, achteruitEscalatie = 0;
    float       ontsnapX = 1e9f, ontsnapY = 1e9f;

    CommandKeepAlive ka(uart);
    ScanMatcher      scanMatcher;  // ICP scan-to-scan matching

    float lastRanges[360] = {};
    bool  heeftRanges     = false;

    // Wacht tot Pico opgestart is
    usleep(1200000);
    for (int i = 0; i < 5; ++i) { uart.StuurStop(); usleep(20000); }
    tcflush(uart.GetFd(), TCIFLUSH);
    for (int i = 0; i < 15; ++i) { uart.LeesData(); usleep(10000); }

    // Wacht op eerste geldige LIDAR-scan
    while (!heeftRanges && running) {
        if (lidar.Update()) {
            for (int a = 0; a < 360; ++a) lastRanges[a] = lidar.GetDistance(a).distance;
            heeftRanges = true;
        } else usleep(100000);
    }

    ka.Start();

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        uart.LeesData();
        SensorData sens = uart.GetSensorData();
        if (sens.geldig) {
            if (!imuGenulld) { imuOffset = sens.yawGraden; imuGenulld = true; }
            omegaDegS = (sens.speedRechts - sens.speedLinks) / 235.0f * (180.0f / M_PI);
            loc.Predict(sens.speedLinks, sens.speedRechts, DT);
            loc.UpdateIMU(sens.yawGraden - imuOffset, DT);
            huidigeImuYaw = sens.yawGraden - imuOffset;

            if (!beginPuntVergrendeld) {
                beginPunt = Position(loc.GetX(), loc.GetY(), loc.GetTheta());
                beginPuntVergrendeld = true;
            }
        }

        Position pos(loc.GetX(), loc.GetY(), huidigeImuYaw);

        bool nieuweScan = false;
        if (lidar.Update()) {
            float angles[360];
            for (int a = 0; a < 360; ++a) {
                lastRanges[a] = lidar.GetDistance(a).distance;
                angles[a]     = static_cast<float>(a);
            }

            // ICP scan matching: corrigeer positie voor we in kaart schrijven
            IcpResult icp = scanMatcher.Match(lastRanges, huidigeImuYaw);
            if (icp.valid) {
                loc.ApplyIcpCorrection(icp.dx, icp.dy, icp.dtheta);
                huidigeImuYaw = NormDeg(huidigeImuYaw + icp.dtheta);
                pos = Position(loc.GetX(), loc.GetY(), huidigeImuYaw);
            } else {
                // ICP mislukt: synchroniseer het ankerpunt met de huidige
                // odometriepositie zodat de volgende geslaagde ICP-match
                // weet waar de robot nu staat.
                loc.SetIcpAnchor();
            }

            mapper.UpdateMotionCorrected(lastRanges, angles, 360, pos, omegaDegS, scanDuurSec);
            heeftRanges = true;
            ++scanCount;
            ++scansSindsHerplan;
            nieuweScan = true;
        }

        ScanAnalysis scan{};
        if (heeftRanges) scan = AnalyzeScan(lastRanges);

        // Vastzit-detectie: als de robot in ontwijkmodus zit maar niet beweegt
        if (ontwijkFase != OntwijkFase::NORMAAL && hoofdModus != HoofdModus::TERUG_HOME) {
            if (vastzitTicks == 0) { vastzitRefX = pos.GetX(); vastzitRefY = pos.GetY(); }
            ++vastzitTicks;
            float beweging = std::hypot(pos.GetX() - vastzitRefX, pos.GetY() - vastzitRefY);
            if (beweging > VASTZIT_BEWEG_MM) {
                vastzitRefX = pos.GetX(); vastzitRefY = pos.GetY(); vastzitTicks = 0;
            }
            if (vastzitTicks >= VASTZIT_TIMEOUT) {
                hoofdModus = HoofdModus::TERUG_HOME;
                ontwijkFase = OntwijkFase::NORMAAL;
                heeftPad = false;
                scansSindsHerplan = HERPLAN_SCANS;
                Path pad = planner.PlanPath(pos, beginPunt, mapper.GetMap());
                if (!pad.IsEmpty()) { navigator.SetPath(pad); mapper.SetWaypoints(pad); heeftPad = true; }
            }
        } else {
            vastzitTicks = 0;
        }

        // Rijlogica
        if (ontwijkFase != OntwijkFase::NORMAAL) {
            HandleObstacleAvoidance(scan, loc, ontwijkFase, doelHoek, draaiRichting,
                vrijrijTicks, achteruitTicks, vastzitTeller_loc, achteruitEscalatie,
                ontsnapX, ontsnapY, ka, LIN_SPEED, scansSindsHerplan);
        } else {
            switch (hoofdModus) {
                case HoofdModus::FRONTIER:
                    HandleFrontierMode(navigator, mapper, pos, nieuweScan, scansSindsHerplan,
                        mislukteTeller, hoofdModus, heeftPad, ontsnapX, ontsnapY, ka, planner,
                        frontierBlacklist, HERPLAN_SCANS, MIN_DEKKING_PCT,
                        MISLUKT_DREMPEL, ONTSNAP_RADIUS, lastRanges, scan.minFront);
                    break;
                case HoofdModus::TERUG_HOME:
                    HandleReturnToHome(navigator, mapper, pos, beginPunt, heeftPad, ka, planner,
                        HOME_DREMPEL_MM, hoofdModus);
                    break;
                case HoofdModus::KLAAR:
                    running = false;
                    break;
            }
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - tStart).count();
        long rest = LOOP_US - elapsed;
        if (rest > 0) usleep(static_cast<useconds_t>(rest));
    }

    mapper.SaveDebugMap("kaart.pgm");
    ka.Stop();
    ka.Shutdown();
    for (int i = 0; i < 10; ++i) { uart.StuurStop(); usleep(50000); }
    lidar.Disconnect();
    return 0;
}

// ─────────────────────────────────────────────────────────────────
// RunMappen
// ─────────────────────────────────────────────────────────────────
static int RunMappen(Pi5UARTHandler& uart, LIDAR& lidar) {
    Localisation loc(235.0f);
    Mapper mapper(5000, 500, 0.03f);
    constexpr float DT = 0.1f, LOOP_US = 100000;
    int scanCount = 0;

    float imuOffset  = 0.0f;
    bool  imuGenulld = false;
    float huidigeImuYaw = 0.0f;

    usleep(1200000);

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        uart.LeesData();
        SensorData sens = uart.GetSensorData();
        if (sens.geldig) {
            if (!imuGenulld) { imuOffset = sens.yawGraden; imuGenulld = true; }
            loc.Predict(sens.speedLinks, sens.speedRechts, DT);
            loc.UpdateIMU(sens.yawGraden - imuOffset, DT);
            huidigeImuYaw = sens.yawGraden - imuOffset;
        }

        if (!lidar.Update()) { usleep(200000); continue; }

        float ranges[360], angles[360];
        for (int a = 0; a < 360; ++a) {
            ranges[a] = lidar.GetDistance(a).distance;
            angles[a] = static_cast<float>(a);
        }

        Position pos(loc.GetX(), loc.GetY(), huidigeImuYaw);
        mapper.Update(ranges, angles, 360, pos);
        ++scanCount;

        mapper.PrintMap(loc.GetX(), loc.GetY(), scanCount, mapper.GetCoverage());

        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - tStart).count();
        long rest = (long)LOOP_US - elapsed;
        if (rest > 0) usleep((useconds_t)rest);
    }

    lidar.Disconnect();
    mapper.SaveDebugMap("kaart.pgm");
    return 0;
}

// ─────────────────────────────────────────────────────────────────
// RunPicoCommunicatie — handmatig besturen + live mappen
//
// Gebruik:
//   w / s        vooruit / achteruit  (+/- LIN_SPEED mm/s)
//   a / d        links / rechts draaien (+/- ANG_SPEED deg/s)
//   spatie        stop
//   q             afsluiten en kaart opslaan
//
// De LIDAR + lokalisatie lopen in dezelfde lus als RunMappen zodat de
// kaart live wordt opgebouwd terwijl je handmatig rijdt. Zo kun je
// stap voor stap uitsluiten of slechte kaartdelen door rijgedrag of
// door de mapping/lokalisatie worden veroorzaakt.
// ─────────────────────────────────────────────────────────────────
static int RunPicoCommunicatie(Pi5UARTHandler& uart, LIDAR& lidar) {
    constexpr float DT       = 0.1f;
    constexpr long  LOOP_US  = 100000;
    constexpr float LIN_SPEED = 200.0f;   // mm/s — bewust lager dan autonoom voor controle
    constexpr float ANG_SPEED =  45.0f;   // deg/s

    Localisation loc(219.0f);
    Mapper       mapper(260, 160, 0.03f);
    ScanMatcher  scanMatcher;

    float imuOffset     = 0.0f;
    bool  imuGenulld    = false;
    float huidigeImuYaw = 0.0f;
    float omegaDegS     = 0.0f;
    int   scanCount     = 0;

    float lastRanges[360] = {};
    bool  heeftRanges     = false;

    // Huidig handmatig commando
    float handLin = 0.0f;
    float handAng = 0.0f;

    CommandKeepAlive ka(uart);

    // Zet terminal in raw mode zodat toetsen direct beschikbaar zijn
    // zonder Enter te hoeven drukken.
    struct termios oudeTermios, nieuweTermios;
    tcgetattr(STDIN_FILENO, &oudeTermios);
    nieuweTermios = oudeTermios;
    nieuweTermios.c_lflag &= ~(ICANON | ECHO); // geen line-buffering, geen echo
    nieuweTermios.c_cc[VMIN]  = 0;             // niet-blokkerende read
    nieuweTermios.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &nieuweTermios);

    usleep(1200000);
    for (int i = 0; i < 5; ++i) { uart.StuurStop(); usleep(20000); }
    tcflush(uart.GetFd(), TCIFLUSH);
    for (int i = 0; i < 15; ++i) { uart.LeesData(); usleep(10000); }

    // Wacht op eerste LIDAR-scan
    while (!heeftRanges && running) {
        if (lidar.Update()) {
            for (int a = 0; a < 360; ++a) lastRanges[a] = lidar.GetDistance(a).distance;
            heeftRanges = true;
        } else usleep(100000);
    }

    ka.Start();

    printf("\n[PICO] Handmatige besturing actief\n");
    printf("  w/s = voor/achteruit  a/d = links/rechts  spatie = stop  q = afsluiten\n\n");

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        // ── Lees toetsenbord (niet-blokkerend) ──────────────────────
        char toets = 0;
        if (read(STDIN_FILENO, &toets, 1) == 1) {
            switch (toets) {
                case 'w': handLin =  LIN_SPEED; handAng =  0.0f;      break;
                case 's': handLin = -LIN_SPEED; handAng =  0.0f;      break;
                case 'a': handLin =  0.0f;      handAng = -ANG_SPEED; break;
                case 'd': handLin =  0.0f;      handAng =  ANG_SPEED; break;
                case ' ': handLin =  0.0f;      handAng =  0.0f;      break;
                case 'q': running = false;                             break;
                default:  break;
            }
            if (toets != 'q') {
                ka.SetCommand(handLin, handAng);
                printf("[PICO] cmd lin=%.0f ang=%.0f\n", handLin, handAng);
            }
        }

        // ── Sensordata + lokalisatie ─────────────────────────────────
        uart.LeesData();
        SensorData sens = uart.GetSensorData();
        if (sens.geldig) {
            if (!imuGenulld) { imuOffset = sens.yawGraden; imuGenulld = true; }
            omegaDegS = (sens.speedRechts - sens.speedLinks) / 219.0f * (180.0f / M_PI);
            loc.Predict(sens.speedLinks, sens.speedRechts, DT);
            loc.UpdateIMU(sens.yawGraden - imuOffset, DT);
            huidigeImuYaw = sens.yawGraden - imuOffset;
        }

        Position pos(loc.GetX(), loc.GetY(), huidigeImuYaw);

        // ── LIDAR + mapping ──────────────────────────────────────────
        if (lidar.Update()) {
            float angles[360];
            for (int a = 0; a < 360; ++a) {
                lastRanges[a] = lidar.GetDistance(a).distance;
                angles[a]     = static_cast<float>(a);
            }

            IcpResult icp = scanMatcher.Match(lastRanges, huidigeImuYaw);
            if (icp.valid) {
                loc.ApplyIcpCorrection(icp.dx, icp.dy, icp.dtheta);
                huidigeImuYaw = NormDeg(huidigeImuYaw + icp.dtheta);
                pos = Position(loc.GetX(), loc.GetY(), huidigeImuYaw);
            }

            constexpr float scanDuurSec = 0.1f;
            mapper.UpdateMotionCorrected(lastRanges, angles, 360, pos, omegaDegS, scanDuurSec);
            ++scanCount;

            mapper.PrintMap(loc.GetX(), loc.GetY(), scanCount, mapper.GetCoverage());
        }

        // ── Loop timing ──────────────────────────────────────────────
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - tStart).count();
        long rest = LOOP_US - static_cast<long>(elapsed);
        if (rest > 0) usleep(static_cast<useconds_t>(rest));
    }

    // Herstel terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &oudeTermios);

    ka.Stop();
    ka.Shutdown();
    for (int i = 0; i < 10; ++i) { uart.StuurStop(); usleep(50000); }
    lidar.Disconnect();
    mapper.SaveDebugMap("kaart.pgm");
    printf("[PICO] Kaart opgeslagen als kaart.pgm\n");
    return 0;
}

// ─────────────────────────────────────────────────────────────────
// Menu + main
// ─────────────────────────────────────────────────────────────────
enum class MenuKeuze { MAPPEN, PICO_COMMUNICATIE, RIJDEN_EN_MAPPEN, STOPPEN };

static MenuKeuze VraagMenuKeuze() {
    while (true) {
        std::cout << "\n"
                  << "╔══════════════════════════════════════╗\n"
                  << "║ ROBOT CONTROLLER Pi5                 ║\n"
                  << "╠══════════════════════════════════════╣\n"
                  << "║ 1. Mappen                            ║\n"
                  << "║ 2. Pico communiceren                 ║\n"
                  << "║ 3. Autonoom rijden + mappen          ║\n"
                  << "║ 4. Stoppen                           ║\n"
                  << "╚══════════════════════════════════════╝\n"
                  << "Keuze: ";
        std::string invoer; std::getline(std::cin, invoer);
        if (invoer == "1") return MenuKeuze::MAPPEN;
        if (invoer == "2") return MenuKeuze::PICO_COMMUNICATIE;
        if (invoer == "3") return MenuKeuze::RIJDEN_EN_MAPPEN;
        if (invoer == "4") return MenuKeuze::STOPPEN;
        std::cout << "Ongeldige keuze.\n";
    }
}

int main() {
    signal(SIGINT, onSignal);

    Pi5UARTHandler uart("/dev/ttyAMA10");
    LIDAR lidar("/dev/ttyUSB0", 460800);

    g_uart = &uart;
    if (!uart.Open()) { std::cerr << "UART niet beschikbaar.\n"; return 1; }

    if (!uart.RebootPico()) std::cout << "Pico reboot overgeslagen (al actief).\n";
    else usleep(1500000);

    switch (VraagMenuKeuze()) {
        case MenuKeuze::MAPPEN:
            if (!lidar.Connect()) { std::cerr << "LIDAR niet beschikbaar.\n"; return 1; }
            return RunMappen(uart, lidar);
        case MenuKeuze::PICO_COMMUNICATIE:
            if (!lidar.Connect()) { std::cerr << "LIDAR niet beschikbaar.\n"; return 1; }
            return RunPicoCommunicatie(uart, lidar);
        case MenuKeuze::RIJDEN_EN_MAPPEN:
            if (!lidar.Connect()) { std::cerr << "LIDAR niet beschikbaar.\n"; return 1; }
            return RunRijdenEnMappen(uart, lidar);
        case MenuKeuze::STOPPEN:
            return 0;
    }
    return 0;
}