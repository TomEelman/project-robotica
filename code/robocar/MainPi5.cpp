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
#include <vector>
#include <unistd.h>
#include <termios.h>

// Zet op 1 om de LIDAR-kaart + debug-printf's te tonen, 0 om ze te verbergen
#define DEBUG_PRINT 0

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
    float /*ontsnapX*/, float /*ontsnapY*/, CommandKeepAlive& ka, PathPlanner& planner,
    std::vector<BlacklistItem>& frontierBlacklist, int HERPLAN_SCANS, int /*MIN_DEKKING_PCT*/,
    int MISLUKT_DREMPEL, float /*ONTSNAP_RADIUS*/, const float* lastRanges,
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
    float /*ontsnapX*/, float /*ontsnapY*/, CommandKeepAlive& ka, PathPlanner& planner,
    std::vector<BlacklistItem>& frontierBlacklist, int HERPLAN_SCANS, int /*MIN_DEKKING_PCT*/,
    int MISLUKT_DREMPEL, float /*ONTSNAP_RADIUS*/, const float* lastRanges,
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
#if DEBUG_PRINT
            printf("[MAIN] Navigator blocked -> replan\n");
#endif
            Position bt = navigator.GetCurrentTarget();
            VoegToeAanBlacklist(frontierBlacklist, bt.GetX(), bt.GetY());
            navigator.ResetBlock();
            heeftPad = false;
            scansSindsHerplan = HERPLAN_SCANS;
        }

        if (navigator.IsBlocked()) {
#if DEBUG_PRINT
            printf("[MAIN] Navigator blocked -> wacht op stabiele lokalisatie\n");
#endif
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

            // ── Kamer-dekkingscheck ───────────────────────────────────
            // outerWallPct   : % van de buitenste rand (5-cel-band) dat
            //                  als muur geclassificeerd is → muren rondom gevonden.
            // interiorPct    : % bekende cellen BINNEN die rand → interieur gemapt.
            // relCoveragePct : % van de bounding box dat bekend is (eerlijk getal,
            //                  vervangt de gebroken GetCoverage() die door het
            //                  hele grid deelt en zo veel te laag uitkomt).
            constexpr float OUTER_WALL_MIN  = 25.0f;  // % buitenste rand = muur
            constexpr float INTERIOR_MIN    = 80.0f;  // % interieur bekend
            float outerWallPct, interiorPct, relCoveragePct;
            mapper.GetRoomCoverage(outerWallPct, interiorPct, relCoveragePct);

            bool kaartKlaar = (outerWallPct  >= OUTER_WALL_MIN &&
                               interiorPct   >= INTERIOR_MIN);

#if DEBUG_PRINT
            printf("[MAIN] FRONTIER rand=%.0f%% int=%.0f%% rel=%.0f%% mislukt=%d%s\n",
                   outerWallPct, interiorPct, relCoveragePct, mislukteTeller,
                   kaartKlaar ? " *** KAART KLAAR ***" : "");
#endif

            if (kaartKlaar || mislukteTeller >= MISLUKT_DREMPEL) {
                hoofdModus = HoofdModus::TERUG_HOME;
                heeftPad = false;
                mislukteTeller = 0;
#if DEBUG_PRINT
                printf("[MAIN] Kaart gemapped (rand=%.0f%% int=%.0f%%) -> TERUG_HOME\n",
                       outerWallPct, interiorPct);
#endif
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
                        // Doel ligt in een obstakel of marge — blacklist het en stop.
                        // Vooruit rijden hier stuurde de robot recht terug in het obstakel.
                        // Bij de volgende scan kiest KiesFrontierDoel een ander doel.
                        ++mislukteTeller;
                        VoegToeAanBlacklist(frontierBlacklist, doel.GetX(), doel.GetY());
                        ka.Stop();
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
        printf("[MAIN] Kaart gemapped!\n");
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
                float hoek = static_cast<float>(std::atan2(-dy, -dx)) * (180.0f / static_cast<float>(M_PI));
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
// EncoderMetTeken — corrigeert altijd-positieve encoder-waarden
//
// De Pico-encoders geven de rijsnelheid als absolute waarde (magnitude),
// ongeacht de rijrichting. Deze functie bepaalt het teken van elk wiel
// op basis van het laatste gestuurde commando.
//
// Formule zelfde als Drive.cpp ExecuteLinear:
//   vLeft_cmd  = lin + omega_rad * wheelbase/2
//   vRight_cmd = lin - omega_rad * wheelbase/2
//
// Het teken van vLeft_cmd / vRight_cmd geeft de rijrichting van elk wiel.
// Bij transitie (ramp loopt nog) kan het teken 1 tick te vroeg omslaan,
// maar dat is verwaarloosbaar t.o.v. de permanente drift bij geen fix.
// ─────────────────────────────────────────────────────────────────
static void EncoderMetTeken(float cmdLin, float cmdAng,
                             float& vLeft, float& vRight)
{
    constexpr float HALF_BASE = 219.0f / 2.0f;
    constexpr float DEG2RAD   = static_cast<float>(M_PI) / 180.0f;

    float omegaRad  = cmdAng * DEG2RAD;
    float cmdVLeft  = cmdLin + omegaRad * HALF_BASE;
    float cmdVRight = cmdLin - omegaRad * HALF_BASE;

    vLeft  = (cmdVLeft  >= 0.0f ? 1.0f : -1.0f) * std::fabs(vLeft);
    vRight = (cmdVRight >= 0.0f ? 1.0f : -1.0f) * std::fabs(vRight);
}

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

    float linSpeed       = 0.0f;  // gemiddelde lineaire snelheid voor ICP-check

    // Encoder-seeding voor ICP: onthoud EKF-positie bij de vorige scan zodat
    // we de geschatte verplaatsing als initiële gok aan Match() kunnen meegeven.
    float lastScanX      = 0.0f, lastScanY = 0.0f;
    bool  hasLastScanPos = false;

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

        bool versData = uart.LeesData();   // true = nieuw Pico-pakket ontvangen
        SensorData sens = uart.GetSensorData();
        if (sens.geldig) {
            if (!imuGenulld) { imuOffset = sens.yawGraden; imuGenulld = true; }

            // Teken herstellen voor beide wielen (encoders zijn altijd positief).
            float vL = sens.speedLinks, vR = sens.speedRechts;
            EncoderMetTeken(ka.GetLin(), ka.GetAng(), vL, vR);

            // omegaDegS op basis van gecorrigeerde snelheden (voor mapper).
            omegaDegS = (vR - vL) / 219.0f * (180.0f / static_cast<float>(M_PI));
            linSpeed  = 0.5f * (vL + vR);  // bijhouden voor ICP-draai-check


            bool beweegt = (sens.speedLinks != 0.0f || sens.speedRechts != 0.0f);
            // Predict/UpdateIMU ALLEEN op vers pakket — voorkomt stale-data drift.
            if (versData && beweegt) {
                loc.Predict(vL, vR, DT);
                loc.UpdateIMU(sens.yawGraden - imuOffset, DT);
            }

            huidigeImuYaw = sens.yawGraden - imuOffset;

            if (!beginPuntVergrendeld) {
                beginPunt = Position(loc.GetX(), loc.GetY(), loc.GetTheta());
                beginPuntVergrendeld = true;
            }
        }

        Position pos(loc.GetX(), loc.GetY(), loc.GetTheta()); // BUG4 FIX: EKF-theta

        bool nieuweScan = false;
        if (lidar.Update()) {
            float angles[360];
            for (int a = 0; a < 360; ++a) {
                lastRanges[a] = lidar.GetDistance(a).distance;
                angles[a]     = static_cast<float>(a);
            }

            // ICP scan matching: corrigeer positie voor we in kaart schrijven.
            // Geef de encoder-verplaatsing mee als initiële gok zodat ICP
            // al dicht bij het goede lokale minimum begint.
            float encDx = 0.0f, encDy = 0.0f;
            if (hasLastScanPos) {
                encDx = loc.GetX() - lastScanX;
                encDy = loc.GetY() - lastScanY;
            }
            IcpResult icp = scanMatcher.Match(lastRanges, huidigeImuYaw,
                                              encDx, encDy);
            // ICP overslaan tijdens draaien: de roterende puntenwolk geeft
            // willekeurige translaties terug die de positie laten springen.
            constexpr float ICP_LIN_MIN = 20.0f;
            constexpr float ICP_OMEGA_MAX = 8.0f;  // deg/s
            
            if (std::fabs(linSpeed) > ICP_LIN_MIN &&
    std::fabs(omegaDegS) < ICP_OMEGA_MAX && icp.valid) {
                loc.ApplyIcpCorrection(icp.dx, icp.dy, icp.dtheta);
                // BUG3 FIX: huidigeImuYaw NIET optellen met icp.dtheta
                pos = Position(loc.GetX(), loc.GetY(), loc.GetTheta()); // BUG4 FIX
            } else {
                loc.SetIcpAnchor();
            }

            // Bewaar huidige EKF-positie voor encoder-seeding van de volgende ICP-ronde.
            lastScanX      = loc.GetX();
            lastScanY      = loc.GetY();
            hasLastScanPos = true;

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
// RunRijdenEnMappenwf — autonoom rijden + mappen via wall following.
// Zelfde mapping/lokalisatie-pijplijn (ICP + motion-corrected mapping)
// als RunRijdenEnMappen, maar de navigatie houdt de rechter muur aan
// via Navigator::ComputeWallCommand i.p.v. frontier/Wavefront.
// ─────────────────────────────────────────────────────────────────
static int RunRijdenEnMappenwf(Pi5UARTHandler& uart, LIDAR& lidar) {
    Localisation loc(219.0f);
    Mapper mapper(500, 500, 0.03f);

    constexpr float DT          = 0.1f;
    constexpr long  LOOP_US     = 100000;
    constexpr float scanDuurSec = 0.1f;

    int   scanCount     = 0;
    float imuOffset     = 0.0f;
    bool  imuGenulld    = false;
    float huidigeImuYaw = 0.0f;
    float omegaDegS     = 0.0f;

    Navigator   navigator;
    navigator.ResetWallFollower();

    CommandKeepAlive ka(uart);
    ScanMatcher      scanMatcher;  // ICP scan-to-scan matching

    float linSpeed       = 0.0f;  // gemiddelde lineaire snelheid voor ICP-check

    // Encoder-seeding voor ICP: onthoud EKF-positie bij de vorige scan zodat
    // we de geschatte verplaatsing als initiele gok aan Match() kunnen meegeven.
    float lastScanX      = 0.0f, lastScanY = 0.0f;
    bool  hasLastScanPos = false;

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

        bool versData = uart.LeesData();   // true = nieuw Pico-pakket ontvangen
        SensorData sens = uart.GetSensorData();
        if (sens.geldig) {
            if (!imuGenulld) { imuOffset = sens.yawGraden; imuGenulld = true; }

            // Teken herstellen voor beide wielen (encoders zijn altijd positief).
            float vL = sens.speedLinks, vR = sens.speedRechts;
            EncoderMetTeken(ka.GetLin(), ka.GetAng(), vL, vR);

            // omegaDegS op basis van gecorrigeerde snelheden (voor mapper).
            omegaDegS = (vR - vL) / 219.0f * (180.0f / static_cast<float>(M_PI));
            linSpeed  = 0.5f * (vL + vR);  // bijhouden voor ICP-draai-check

            bool beweegt = (sens.speedLinks != 0.0f || sens.speedRechts != 0.0f);

            // Predict/UpdateIMU ALLEEN aanroepen als er een vers pakket is én de
            // robot beweegt. Zonder versData-check zou dezelfde snelheid opnieuw
            // geïntegreerd worden (stale-data drift).
            if (versData && beweegt) {
                loc.Predict(vL, vR, DT);
                loc.UpdateIMU(sens.yawGraden - imuOffset, DT);
            }

            huidigeImuYaw = sens.yawGraden - imuOffset;

            if (!versData && beweegt) {
#if DEBUG_PRINT
                printf("[UART] geen vers pakket deze tick — Predict overgeslagen\n");
#endif
            }
        }

        Position pos(loc.GetX(), loc.GetY(), loc.GetTheta()); // BUG4 FIX

        if (lidar.Update()) {
            float angles[360];
            for (int a = 0; a < 360; ++a) {
                lastRanges[a] = lidar.GetDistance(a).distance;
                angles[a]     = static_cast<float>(a);
            }

            // ICP scan matching: corrigeer positie voor we in kaart schrijven.
            // Geef de encoder-verplaatsing mee als initiele gok zodat ICP
            // al dicht bij het goede lokale minimum begint.
            float encDx = 0.0f, encDy = 0.0f;
            if (hasLastScanPos) {
                encDx = loc.GetX() - lastScanX;
                encDy = loc.GetY() - lastScanY;
            }
            IcpResult icp = scanMatcher.Match(lastRanges, huidigeImuYaw,
                                              encDx, encDy);
            constexpr float ICP_LIN_MIN = 20.0f;
            if (std::fabs(linSpeed) > ICP_LIN_MIN && icp.valid) {
                loc.ApplyIcpCorrection(icp.dx, icp.dy, icp.dtheta);
                // BUG3 FIX: huidigeImuYaw niet optellen met icp.dtheta
                pos = Position(loc.GetX(), loc.GetY(), loc.GetTheta()); // BUG4 FIX
            } else {
                loc.SetIcpAnchor();
            }

            lastScanX      = loc.GetX();
            lastScanY      = loc.GetY();
            hasLastScanPos = true;

            mapper.UpdateMotionCorrected(lastRanges, angles, 360, pos, omegaDegS, scanDuurSec);
            heeftRanges = true;
            ++scanCount;
        }

        // Rijlogica: volg de rechter muur. ComputeWallCommand handelt
        // binnen-/buitenbochten en open ruimte zelf af op basis van de scan.
        WallResult wf = navigator.ComputeWallCommand(lastRanges);
        ka.SetCommand(wf.cmd.GetLinVelocity(), wf.cmd.GetAngVelocity());

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
    Localisation loc(219.0f); // BUG2 FIX: consistent 219mm
    Mapper mapper(5000, 500, 0.03f);
    constexpr float DT = 0.1f, LOOP_US = 100000;
    int scanCount = 0;

    float imuOffset  = 0.0f;
    bool  imuGenulld = false;

    usleep(1200000);

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        uart.LeesData();
        SensorData sens = uart.GetSensorData();
        if (sens.geldig) {
            if (!imuGenulld) { imuOffset = sens.yawGraden; imuGenulld = true; }
            loc.Predict(sens.speedLinks, sens.speedRechts, DT);
            loc.UpdateIMU(sens.yawGraden - imuOffset, DT);
        }

        if (!lidar.Update()) { usleep(200000); continue; }

        float ranges[360], angles[360];
        for (int a = 0; a < 360; ++a) {
            ranges[a] = lidar.GetDistance(a).distance;
            angles[a] = static_cast<float>(a);
        }

        Position pos(loc.GetX(), loc.GetY(), loc.GetTheta()); // BUG4 FIX
        mapper.Update(ranges, angles, 360, pos);
        ++scanCount;

#if DEBUG_PRINT
        mapper.PrintMap(loc.GetX(), loc.GetY(), scanCount, mapper.GetCoverage());
#endif

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
// Terminal helpers — raw mode voor directe toetsdetectie
// ─────────────────────────────────────────────────────────────────

// Schakelt stdin in raw mode (geen Enter nodig) of terug naar normaal.
// Bewaart de originele termios de eerste keer zodat we altijd netjes
// kunnen herstellen, ook na meerdere aan/uit-cycli.
static void SetTerminalRaw(bool raw) {
    static struct termios orig;
    static bool saved = false;
    if (!saved) { tcgetattr(STDIN_FILENO, &orig); saved = true; }
    if (raw) {
        struct termios t = orig;
        t.c_lflag &= ~(ICANON | ECHO);
        t.c_cc[VMIN]  = 0;   // niet-blokkerend lezen
        t.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig);
        tcflush(STDIN_FILENO, TCIFLUSH);  // gooi eventuele raw-invoer weg
    }
}

// Leest één toets zonder te blokkeren. Geeft de char-waarde terug of -1.
static int PollKey() {
    char c;
    return (read(STDIN_FILENO, &c, 1) == 1) ? static_cast<int>((unsigned char)c) : -1;
}

// ─────────────────────────────────────────────────────────────────
// RunPicoCommunicatie — interactief rijden + live mappen
//
// Type een commando + Enter → robot rijdt tot je SPATIE drukt.
// SPATIE = nieuw commando invoeren.
// q (tijdens rijden, zonder Enter) of q + Enter (bij prompt) = stoppen
// en terug naar het hoofdmenu.
// ─────────────────────────────────────────────────────────────────
static int RunPicoCommunicatie(Pi5UARTHandler& uart, LIDAR& lidar) {
    constexpr float DT          = 0.1f;
    constexpr long  LOOP_US     = 100000;
    constexpr float scanDuurSec = 0.1f;

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

    // Encoder-seeding voor ICP: onthoud de EKF-positie bij de vorige scan,
    // zodat we de odometrie-verplaatsing als initiële gok aan Match() geven.
    float lastScanX      = 0.0f, lastScanY = 0.0f;
    bool  hasLastScanPos = false;

    CommandKeepAlive ka(uart);

    usleep(1200000);
    for (int i = 0; i < 5; ++i) { uart.StuurStop(); usleep(20000); }
    tcflush(uart.GetFd(), TCIFLUSH);
    for (int i = 0; i < 15; ++i) { uart.LeesData(); usleep(10000); }

    while (!heeftRanges && running) {
        if (lidar.Update()) {
            for (int a = 0; a < 360; ++a) lastRanges[a] = lidar.GetDistance(a).distance;
            heeftRanges = true;
        } else usleep(100000);
    }

    ka.Start();

    printf("\n[PICO] Interactieve rijmodus\n");
    printf("  Commando's: vooruit  achteruit  links  rechts  bochtl  bochtr  stop\n");
    printf("  SPATIE = nieuw commando invoeren  |  q = stoppen en terug naar menu\n\n");

    bool stopGevraagd = false;

    while (running && !stopGevraagd) {
        // ── Invoerfase: normale terminal, wacht op Enter ────────────────────
        SetTerminalRaw(false);
        ka.SetCommand(0.0f, 0.0f);

        printf("commando> ");
        fflush(stdout);

        std::string invoer;
        if (!std::getline(std::cin, invoer)) break;

        // Isoleer eerste token (naam van het commando)
        size_t b = invoer.find_first_not_of(" \t");
        if (b == std::string::npos) continue;
        size_t e = invoer.find_first_of(" \t", b);
        std::string naam = invoer.substr(b, e == std::string::npos ? e : e - b);

        if (naam == "q" || naam == "Q") break;

        float rijLin, rijAng;
        if      (naam == "vooruit")   { rijLin =  278.0f; rijAng =   0.0f; }
        else if (naam == "achteruit") { rijLin = -278.0f; rijAng =   0.0f; }
        else if (naam == "links")     { rijLin =    0.0f; rijAng = -40.0f; }
        else if (naam == "rechts")    { rijLin =    0.0f; rijAng =  40.0f; }
        else if (naam == "bochtl")    { rijLin =   20.0f; rijAng = -40.0f; }
        else if (naam == "bochtr")    { rijLin =   20.0f; rijAng =  40.0f; }
        else if (naam == "stop")      { rijLin =    0.0f; rijAng =   0.0f; }
        else {
            printf("[PICO] Onbekend commando '%s'\n", naam.c_str());
            continue;
        }

        /*printf("  → %s  (lin=%.0f ang=%.0f)  |  SPATIE = ander commando  |  q = stoppen\n",
               naam.c_str(), rijLin, rijAng);
*/
        ka.SetCommand(rijLin, rijAng);

        // ── Rijfase: sensor/mapping loop + wacht op SPATIE of q ────────────
        SetTerminalRaw(true);

        while (running && !stopGevraagd) {
            auto tStart = std::chrono::steady_clock::now();

            // Sensor + lokalisatie (alleen bijwerken tijdens beweging)
            uart.LeesData();
            SensorData sens = uart.GetSensorData();
            bool  beweegt      = false;
            float linSpeedPico = 0.0f;  // voor ICP-draai-check
            if (sens.geldig) {
                if (!imuGenulld) { imuOffset = sens.yawGraden; imuGenulld = true; }

                // Teken herstellen voor beide wielen (encoders zijn altijd positief).
                float vL = sens.speedLinks, vR = sens.speedRechts;
                EncoderMetTeken(ka.GetLin(), ka.GetAng(), vL, vR);

                // omegaDegS op basis van gecorrigeerde snelheden (voor mapper).
                omegaDegS = (vR - vL) / 219.0f * (180.0f / static_cast<float>(M_PI));
                linSpeedPico   = 0.5f * (vL + vR);

                bool versData = uart.LeesData();   // true = nieuw Pico-pakket ontvangen
                beweegt = (sens.speedLinks != 0.0f || sens.speedRechts != 0.0f);
                if (versData && beweegt) {
                    loc.Predict(vL, vR, DT);
                    loc.UpdateIMU(sens.yawGraden - imuOffset, DT);
                }
                huidigeImuYaw = sens.yawGraden - imuOffset;
            }

            // LIDAR + mapping (alleen tijdens beweging)
            if (lidar.Update()) {
                float angles[360];
                for (int a = 0; a < 360; ++a) {
                    lastRanges[a] = lidar.GetDistance(a).distance;
                    angles[a]     = static_cast<float>(a);
                }
                if (beweegt) {
                    Position pos(loc.GetX(), loc.GetY(), loc.GetTheta()); // Bug4: EKF-theta
                    float encDx = 0.0f, encDy = 0.0f;
                    if (hasLastScanPos) {
                        encDx = loc.GetX() - lastScanX;
                        encDy = loc.GetY() - lastScanY;
                    }
                    IcpResult icp = scanMatcher.Match(lastRanges, huidigeImuYaw, encDx, encDy);
                    constexpr float ICP_LIN_MIN = 20.0f;
                    if (std::fabs(linSpeedPico) > ICP_LIN_MIN && icp.valid) {
                        loc.ApplyIcpCorrection(icp.dx, icp.dy, icp.dtheta);
                        // Bug3: huidigeImuYaw NIET optellen met icp.dtheta
                        pos = Position(loc.GetX(), loc.GetY(), loc.GetTheta()); // Bug4
                    } else {
                        loc.SetIcpAnchor();
                    }
                    lastScanX = loc.GetX(); lastScanY = loc.GetY(); hasLastScanPos = true;
                    mapper.UpdateMotionCorrected(lastRanges, angles, 360, pos, omegaDegS, scanDuurSec);
                    ++scanCount;
#if DEBUG_PRINT
                    mapper.PrintMap(loc.GetX(), loc.GetY(), scanCount, mapper.GetCoverage());
#endif
                }
            }

            // Toetsdetectie (niet-blokkerend)
            int ch = PollKey();
            if (ch == ' ') {
                // Spatie → stop en vraag een nieuw commando
                ka.SetCommand(0.0f, 0.0f);
                printf("\n");
                break;
            }
            if (ch == 'q' || ch == 'Q') {
                // q → direct stoppen en terug naar menu
                ka.SetCommand(0.0f, 0.0f);
                stopGevraagd = true;
                break;
            }

            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - tStart).count();
            long rest = LOOP_US - static_cast<long>(elapsed);
            if (rest > 0) usleep(static_cast<useconds_t>(rest));
        }
    }

    SetTerminalRaw(false);
    ka.SetCommand(0.0f, 0.0f);
    ka.Stop();
    ka.Shutdown();
    for (int i = 0; i < 10; ++i) { uart.StuurStop(); usleep(50000); }
    lidar.Disconnect();
    mapper.SaveDebugMap("kaart.pgm");
    //printf("[PICO] Kaart opgeslagen als kaart.pgm\n");
    return 0;
}

enum class MenuKeuze { MAPPEN, PICO_COMMUNICATIE, PATHFINDING, RIJDEN_EN_MAPPEN };

static MenuKeuze VraagMenuKeuze() {
    while (true) {
        std::cout << "\n"
                  << "╔══════════════════════════════════════╗\n"
                  << "║ ROBOT CONTROLLER Pi5                 ║\n"
                  << "╠══════════════════════════════════════╣\n"
                  << "║ 1. Mappen                            ║\n"
                  << "║ 2. Pico communiceren                 ║\n"
                  << "║ 3. Pathfinding                       ║\n"
                  << "║ 4. Autonoom rijden + mappen          ║\n"
                  << "╚══════════════════════════════════════╝\n"
                  << "Keuze: ";
        std::string invoer; std::getline(std::cin, invoer);
        if (invoer == "1") return MenuKeuze::MAPPEN;
        if (invoer == "2") return MenuKeuze::PICO_COMMUNICATIE;
        if (invoer == "3") return MenuKeuze::PATHFINDING;
        if (invoer == "4") return MenuKeuze::RIJDEN_EN_MAPPEN;
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
        case MenuKeuze::PATHFINDING:
            if (!lidar.Connect()) { std::cerr << "LIDAR niet beschikbaar.\n"; return 1; }
            return RunRijdenEnMappen(uart, lidar);
        case MenuKeuze::RIJDEN_EN_MAPPEN:
            if (!lidar.Connect()) { std::cerr << "LIDAR niet beschikbaar.\n"; return 1; }
            return RunRijdenEnMappenwf(uart, lidar);
    }
    return 0;
}