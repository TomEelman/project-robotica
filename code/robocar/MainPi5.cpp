#include "MainPi5.h"
#include "LIDAR.h"
#include "Mapper.h"
#include "Localisation.h"
#include "PathPlanner.h"
#include "Navigator.h"
#include "ExplorationPlanner.h"
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
enum class HoofdModus { MUUR_VOLGEN, FRONTIER, TERUG_HOME, KLAAR };
enum class OntwijkFase { NORMAAL, DRAAIEN, VRIJRIJDEN, ACHTERUIT };

// ─────────────────────────────────────────────────────────────────
// Helper declaraties
// ─────────────────────────────────────────────────────────────────
static bool HandleObstacleAvoidance(const ScanAnalyse& scan, Localisation& loc,
    OntwijkFase& ontwijkFase, float& doelHoek, float& draaiRichting,
    int& vrijrijTicks, int& achteruitTicks, int& vastzitTeller, int& achteruitEscalatie,
    float& ontsnapX, float& ontsnapY, CommandKeepAlive& ka, float LIN_SPEED, int& scansSindsHerplan);

static void HandleWallFollowing(Navigator& navigator, const float* lastRanges, bool nieuweScan,
    int& wallScans, HoofdModus& hoofdModus, PathPlanner& planner,
    std::vector<BlacklistItem>& frontierBlacklist, Position pos, CommandKeepAlive& ka,
    int HERPLAN_SCANS, int MIN_OMLOOP_SCANS, int MIN_DEKKING_PCT, int FRONTIER_DREMPEL, Mapper& mapper);

static void HandleFrontierMode(Navigator& navigator, Mapper& mapper, Position pos, bool nieuweScan,
    int& scansSindsHerplan, int& mislukteTeller, HoofdModus& hoofdModus, bool& heeftPad,
    float ontsnapX, float ontsnapY, CommandKeepAlive& ka, PathPlanner& planner,
    std::vector<BlacklistItem>& frontierBlacklist, int HERPLAN_SCANS, int MIN_DEKKING_PCT,
    int FRONTIER_DREMPEL, int MISLUKT_DREMPEL, float ONTSNAP_RADIUS, const float* lastRanges);

static void HandleReturnToHome(Navigator& navigator, Mapper& mapper, Position pos, Position beginPunt,
    bool& heeftPad, CommandKeepAlive& ka, PathPlanner& planner, float HOME_DREMPEL_MM);

// ─────────────────────────────────────────────────────────────────
// Helper implementaties
// ─────────────────────────────────────────────────────────────────
static bool HandleObstacleAvoidance(const ScanAnalyse& scan, Localisation& loc,
    OntwijkFase& ontwijkFase, float& doelHoek, float& draaiRichting,
    int& vrijrijTicks, int& achteruitTicks, int& vastzitTeller, int& achteruitEscalatie,
    float& ontsnapX, float& ontsnapY, CommandKeepAlive& ka, float LIN_SPEED, int& scansSindsHerplan) {

    if (ontwijkFase == OntwijkFase::DRAAIEN) {
        float fout = NormDeg(doelHoek - loc.GetTheta());
        if (std::fabs(fout) < 8.0f) {
            ontwijkFase = OntwijkFase::VRIJRIJDEN;
            vrijrijTicks = (scan.minVoor < 600.0f ? 10 : 25) + achteruitEscalatie * 8;
            scansSindsHerplan = 15;
            ka.SetCommand(LIN_SPEED, 0.0f);
        } else if (scan.staat >= 3) {
            draaiRichting = -draaiRichting;
            doelHoek = NormDeg(loc.GetTheta() - draaiRichting * 90.0f);
            ka.SetCommand(0.0f, draaiRichting * 40.0f);
        } else {
            ka.SetCommand(0.0f, draaiRichting * 40.0f);
        }
    } else if (ontwijkFase == OntwijkFase::VRIJRIJDEN) {
        if (scan.staat >= 3) {
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
        if (scan.minAchter < 300.0f || --achteruitTicks <= 0) {
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

static void HandleWallFollowing(Navigator& navigator, const float* lastRanges, bool nieuweScan,
    int& wallScans, HoofdModus& hoofdModus, PathPlanner& planner,
    std::vector<BlacklistItem>& frontierBlacklist, Position pos, CommandKeepAlive& ka,
    int HERPLAN_SCANS, int MIN_OMLOOP_SCANS, int MIN_DEKKING_PCT, int FRONTIER_DREMPEL, Mapper& mapper) {

    WallResult wf = navigator.BerekenMuurCommando(lastRanges);
    if (wf.staat != WallStaat::OPEN_RUIMTE) {
        ka.SetCommand(wf.cmd.GetLinVelocity(), wf.cmd.GetAngVelocity());
        if (nieuweScan) {
            ++wallScans;
            if (wallScans >= MIN_OMLOOP_SCANS) {
                int aantalFrontiers = TelFrontiers(mapper);
                int dekking = mapper.GetCoverage();
                if (dekking >= MIN_DEKKING_PCT && aantalFrontiers <= FRONTIER_DREMPEL * 2) {
                    printf("[MAIN] Omloop klaar (%d scans, %d%%) → FRONTIER\n", wallScans, dekking);
                    hoofdModus = HoofdModus::FRONTIER;
                }
            }
        }
    } else {
        ka.SetCommand(wf.cmd.GetLinVelocity(), 0.0f);
        if (nieuweScan) {
            TickBlacklist(frontierBlacklist);
            Position doel = KiesFrontierDoel(mapper, pos, lastRanges, frontierBlacklist);
            if (doel.GetX() != pos.GetX() || doel.GetY() != pos.GetY()) {
                Path pad = planner.PlanPath(pos, doel, mapper.GetMap());
                if (!pad.IsEmpty()) {
                    navigator.SetPath(pad);
                    hoofdModus = HoofdModus::FRONTIER;
                    printf("[MAIN] Open ruimte → frontier (%.0f, %.0f)\n", doel.GetX(), doel.GetY());
                }
            }
        }
    }
}

static void HandleFrontierMode(Navigator& navigator, Mapper& mapper, Position pos, bool nieuweScan,
    int& scansSindsHerplan, int& mislukteTeller, HoofdModus& hoofdModus, bool& heeftPad,
    float ontsnapX, float ontsnapY, CommandKeepAlive& ka, PathPlanner& planner,
    std::vector<BlacklistItem>& frontierBlacklist, int HERPLAN_SCANS, int MIN_DEKKING_PCT,
    int FRONTIER_DREMPEL, int MISLUKT_DREMPEL, float ONTSNAP_RADIUS, const float* lastRanges) {

    WallResult wf = navigator.BerekenMuurCommando(lastRanges);
    if (wf.staat != WallStaat::OPEN_RUIMTE) {
        hoofdModus = HoofdModus::MUUR_VOLGEN;
        heeftPad = false;
        navigator.ResetMuurvolger();
        printf("[MAIN] Muur teruggevonden → MUUR_VOLGEN\n");
        ka.SetCommand(wf.cmd.GetLinVelocity(), wf.cmd.GetAngVelocity());
        return;
    }

    if (heeftPad && !navigator.IsFinished()) {
        navigator.Update(pos);
        DriveCommand cmd = navigator.GetNextCommand(pos, scan.minVoor);
        ka.SetCommand(cmd.GetLinVelocity(), cmd.GetAngVelocity());
    } else {
        if (heeftPad && navigator.IsFinished()) {
            heeftPad = false;
            scansSindsHerplan = HERPLAN_SCANS;
        }
        if (nieuweScan) {
            scansSindsHerplan = 0;
            int aantalFrontiers = TelFrontiers(mapper);
            int dekking = mapper.GetCoverage();

            if (dekking >= MIN_DEKKING_PCT && (aantalFrontiers <= FRONTIER_DREMPEL || mislukteTeller >= MISLUKT_DREMPEL)) {
                hoofdModus = HoofdModus::TERUG_HOME;
                heeftPad = false;
                mislukteTeller = 0;
                printf("[MAIN] Klaar → TERUG_HOME\n");
                Path pad = planner.PlanPath(pos, Position(0.0f, 0.0f, 0.0f), mapper.GetMap());
                if (!pad.IsEmpty()) { navigator.SetPath(pad); heeftPad = true; }
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
                        heeftPad = true;
                        mislukteTeller = 0;
                    } else {
                        ++mislukteTeller;
                        VoegToeAanBlacklist(frontierBlacklist, doel.GetX(), doel.GetY());
                        ka.SetCommand(200.0f, 0.0f);
                    }
                }
            }
        } else {
             // Geen nieuwe scan, geen nieuw commando — ka herhaalt het laatste vanzelf
    (void)0;
            //ka.SetCommand(150.0f, 0.0f);
        }
    }
}

static void HandleReturnToHome(Navigator& navigator, Mapper& mapper, Position pos, Position beginPunt,
    bool& heeftPad, CommandKeepAlive& ka, PathPlanner& planner, float HOME_DREMPEL_MM) {

    float dx = pos.GetX() - beginPunt.GetX();
    float dy = pos.GetY() - beginPunt.GetY();
    if (std::hypot(dx, dy) < HOME_DREMPEL_MM) {
        ka.Stop();
        mapper.SaveDebugMap("kaart.pgm");
        printf("[MAIN] Thuis aangekomen!\n");
    } else {
        if (!heeftPad || navigator.IsFinished()) {
            heeftPad = false;
            Path pad = planner.PlanPath(pos, beginPunt, mapper.GetMap());
            if (!pad.IsEmpty()) {
                navigator.SetPath(pad);
                heeftPad = true;
            } else {
                float hoek = std::atan2(-dy, -dx) * (180.0f / M_PI);
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
    Mapper mapper(260, 160, 0.03f);

    constexpr float DT = 0.1f;
    constexpr long LOOP_US = 100000;
    constexpr float LIN_SPEED = 278.0f;
    constexpr int HERPLAN_SCANS = 15;
    constexpr int MIN_DEKKING_PCT  = 5;
    constexpr int FRONTIER_DREMPEL = 10;
    constexpr int MISLUKT_DREMPEL = 5;
    constexpr float HOME_DREMPEL_MM = 300.0f;
    constexpr float ONTSNAP_RADIUS = 700.0f;
    constexpr int MIN_OMLOOP_SCANS = 300;
    constexpr float scanDuurSec = 0.1f;

    HoofdModus hoofdModus = HoofdModus::MUUR_VOLGEN;
    int scanCount = 0;
    int wallScans = 0;
    int scansSindsHerplan = HERPLAN_SCANS;
    int mislukteTeller = 0;

    constexpr int VASTZIT_TIMEOUT = 400;
    constexpr float VASTZIT_BEWEG_MM = 150.0f;

    int vastzitTicks = 0;
    float vastzitRefX = 0.0f, vastzitRefY = 0.0f;

    float imuOffset = 0.0f;
    bool imuGenulld = false;

    // ── FIX: directe IMU-yaw bijhouden voor gebruik in scan ──────
    float huidigeImuYaw = 0.0f;  // meest recente IMU-yaw, direct uit sensor
    float omegaDegS = 0.0f;

    std::vector<BlacklistItem> frontierBlacklist;
    PathPlanner planner(mapper.GetMap());
    Navigator navigator;
    bool heeftPad = false;

    Position beginPunt(0.0f, 0.0f, 0.0f);
    bool beginPuntVergrendeld = false;

    OntwijkFase ontwijkFase = OntwijkFase::NORMAAL;
    float doelHoek = 0.0f;
    float draaiRichting = 0.0f;
    int vrijrijTicks = 0, achteruitTicks = 0, vastzitTeller_loc = 0, achteruitEscalatie = 0;
    float ontsnapX = 1e9f, ontsnapY = 1e9f;

    // ── FIX: ka.Start() bewust NIET hier — zie hieronder ─────────
    CommandKeepAlive ka(uart);

    float lastRanges[360] = {};
    bool heeftRanges = false;

    // Wacht tot Pico opgestart is en houd motoren actief stil.
    usleep(1200000);
    for (int i = 0; i < 5; ++i) { uart.StuurStop(); usleep(20000); }

    // Gooi alle UART-data weg die tijdens de sleep is opgebouwd,
    // zodat de PID/EMA-filter straks met schone sensorwaarden start.
    // tcflush werkt direct op de file descriptor van de seriële poort.
    tcflush(uart.GetFd(), TCIFLUSH);

    // Lees daarna nog een paar frames leeg zodat GetSensorData()
    // geen stale pakketjes meer teruggeeft.
    for (int i = 0; i < 15; ++i) { uart.LeesData(); usleep(10000); }

    // Wacht op eerste geldige LIDAR-scan vóór we gaan rijden,
    // zodat de obstakels-check meteen klopt bij de eerste loop-iteratie.
    while (!heeftRanges && running) {
        if (lidar.Update()) {
            for (int a = 0; a < 360; ++a) lastRanges[a] = lidar.GetDistance(a).distance;
            heeftRanges = true;
        } else usleep(100000);
    }

    // ── FIX: PAS NU starten — UART-buffer leeg, LIDAR klaar,
    //         PID/filter op de Pico beginnen met een schone lei. ──
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

            // ── FIX: sla de actuele IMU-yaw op voor gebruik bij scan ──
            huidigeImuYaw = sens.yawGraden - imuOffset;

            if (!beginPuntVergrendeld) {
                beginPunt = Position(loc.GetX(), loc.GetY(), loc.GetTheta());
                beginPuntVergrendeld = true;
            }
        }

        // ── FIX: gebruik actuele IMU-yaw voor positie bij de scan ─
        // loc.GetX() en loc.GetY() komen uit de EKF (odometrie),
        // maar voor theta gebruiken we de directe IMU-waarde —
        // die heeft geen loop-vertraging van 100ms.
        Position pos(loc.GetX(), loc.GetY(), huidigeImuYaw);

        bool nieuweScan = false;
        if (lidar.Update()) {
            float angles[360];
            for (int a = 0; a < 360; ++a) {
                lastRanges[a] = lidar.GetDistance(a).distance;
                angles[a] = static_cast<float>(a);
            }
            mapper.UpdateMotionCorrected(lastRanges, angles, 360, pos, omegaDegS, scanDuurSec);
            heeftRanges = true;
            ++scanCount;
            ++scansSindsHerplan;
            nieuweScan = true;
        }

        ScanAnalyse scan{};
        if (heeftRanges) scan = AnalyseerScan(lastRanges);

        // Vastzit-detectie
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
                if (!pad.IsEmpty()) { navigator.SetPath(pad); heeftPad = true; }
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
                case HoofdModus::MUUR_VOLGEN:
                    HandleWallFollowing(navigator, lastRanges, nieuweScan, wallScans, hoofdModus,
                        planner, frontierBlacklist, pos, ka, HERPLAN_SCANS, MIN_OMLOOP_SCANS,
                        MIN_DEKKING_PCT, FRONTIER_DREMPEL, mapper);
                    break;
                case HoofdModus::FRONTIER:
                    HandleFrontierMode(navigator, mapper, pos, nieuweScan, scansSindsHerplan,
                        mislukteTeller, hoofdModus, heeftPad, ontsnapX, ontsnapY, ka, planner,
                        frontierBlacklist, HERPLAN_SCANS, MIN_DEKKING_PCT, FRONTIER_DREMPEL,
                        MISLUKT_DREMPEL, ONTSNAP_RADIUS, lastRanges);
                    break;
                case HoofdModus::TERUG_HOME:
                    HandleReturnToHome(navigator, mapper, pos, beginPunt, heeftPad, ka, planner, HOME_DREMPEL_MM);
                    break;
                case HoofdModus::KLAAR:
                    running = false;
                    break;
            }
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - tStart).count();
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
    Mapper mapper(260, 160, 0.03f);
    constexpr float DT = 0.1f, LOOP_US = 100000;
    int scanCount = 0;

    float imuOffset  = 0.0f;
    bool  imuGenulld = false;

    // ── FIX: directe IMU-yaw bijhouden voor gebruik in scan ──────
    float huidigeImuYaw = 0.0f;

    usleep(1200000);

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        uart.LeesData();
        SensorData sens = uart.GetSensorData();
        if (sens.geldig) {
            if (!imuGenulld) {
                imuOffset  = sens.yawGraden;
                imuGenulld = true;
            }
            loc.Predict(sens.speedLinks, sens.speedRechts, DT);
            loc.UpdateIMU(sens.yawGraden - imuOffset, DT);

            // ── FIX: sla de actuele IMU-yaw op voor gebruik bij scan ──
            huidigeImuYaw = sens.yawGraden - imuOffset;
        }

        if (!lidar.Update()) { usleep(200000); continue; }

        float ranges[360], angles[360];
        for (int a = 0; a < 360; ++a) {
            ranges[a] = lidar.GetDistance(a).distance;
            angles[a] = static_cast<float>(a);
        }

        // ── FIX: gebruik actuele IMU-yaw, niet de vertraagde EKF-theta ──
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

static void RunStuurCommand(CommandKeepAlive& ka) { (void)ka; }
static void RunLeesLive(Pi5UARTHandler& uart) { (void)uart; }
static int RunPicoCommunicatie(Pi5UARTHandler& uart) { (void)uart; return 0; }

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
            return RunPicoCommunicatie(uart);
        case MenuKeuze::RIJDEN_EN_MAPPEN:
            if (!lidar.Connect()) { std::cerr << "LIDAR niet beschikbaar.\n"; return 1; }
            return RunRijdenEnMappen(uart, lidar);
        case MenuKeuze::STOPPEN:
            return 0;
    }
    return 0;
}