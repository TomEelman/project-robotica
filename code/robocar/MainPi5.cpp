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

std::atomic<bool> running{true};
static Pi5UARTHandler* g_uart = nullptr;

static void onSignal(int) {
    running = false;
    if (g_uart && g_uart->IsOpen()) {
        const char* stop = "STOP\nSTOP\nSTOP\n";
        ::write(g_uart->GetFd(), stop, strlen(stop));
    }
}


// ─────────────────────────────────────────────────────────────────
//  CommandKeepAlive
// ─────────────────────────────────────────────────────────────────

CommandKeepAlive::CommandKeepAlive(Pi5UARTHandler& uart)
    : uart(uart), lin(0.0f), ang(0.0f), actief(false)
{}

void CommandKeepAlive::SetCommand(float linearMmS, float angularDegS) {
    std::lock_guard<std::mutex> lk(mtx);
    lin    = linearMmS;
    ang    = angularDegS;
    actief = true;
}

void CommandKeepAlive::Stop() {
    { std::lock_guard<std::mutex> lk(mtx); lin = ang = 0.0f; actief = false; }
    uart.StuurStop();
}

void CommandKeepAlive::Start() {
    threadActief = true;
    worker = std::thread([this]() {
        while (threadActief) {
            {
                std::lock_guard<std::mutex> lk(mtx);
                if (actief && running)       uart.StuurCommand(lin, ang);
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

bool  CommandKeepAlive::IsActief() const { return actief.load(); }
float CommandKeepAlive::GetLin()   const { return lin; }
float CommandKeepAlive::GetAng()   const { return ang; }


// ─────────────────────────────────────────────────────────────────
//  Modus 1: Mappen
// ─────────────────────────────────────────────────────────────────

static int RunMappen(Pi5UARTHandler& uart, LIDAR& lidar) {
    Localisation loc(235.0f);
    Mapper mapper(260, 160, 0.05f);
    constexpr float DT = 0.1f, LOOP_US = 100000;
    int scanCount = 0;

    // ── IMU-offset: nul de yaw bij opstart ───────────────────────
    float imuOffset  = 0.0f;
    bool  imuGenulld = false;

    usleep(1200000);  // spin-up

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
        }

        if (!lidar.Update()) { usleep(200000); continue; }

        float ranges[360], angles[360];
        for (int a = 0; a < 360; ++a) {
            ranges[a] = lidar.GetDistance(a).distance;
            angles[a] = static_cast<float>(a);
        }
        Position pos(loc.GetX(), loc.GetY(), loc.GetTheta());
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
//  Modus 2: Pico communicatie
// ─────────────────────────────────────────────────────────────────

static void RunStuurCommand(CommandKeepAlive& ka) {
    float lin = 0.0f, ang = 0.0f;
    std::cout << "Lineaire snelheid (mm/s): "; std::cin >> lin;
    std::cout << "Hoeksnelheid (deg/s):     "; std::cin >> ang;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    ka.SetCommand(lin, ang);
}

static void RunLeesLive(Pi5UARTHandler& uart) {
    std::atomic<bool> stop{false};
    std::thread t([&]() { std::string d; std::getline(std::cin, d); stop = true; });
    while (!stop) {
        uart.LeesData();
        SensorData s = uart.GetSensorData();
        if (s.geldig)
            printf("ENC L=%7.2f  ENC R=%7.2f  IMU=%7.2f  omega=%6.2f\n",
                   s.speedLinks, s.speedRechts, s.yawGraden, s.hoeksnelheid);
        usleep(100000);
    }
    if (t.joinable()) t.join();
}

static int RunPicoCommunicatie(Pi5UARTHandler& uart) {
    CommandKeepAlive ka(uart);
    ka.Start();

    while (true) {
        std::cout << "\n1=Stuur  2=Sensoren  3=Stop  4=Terug\nKeuze: ";
        std::string keuze; std::getline(std::cin, keuze);
        if      (keuze == "1") RunStuurCommand(ka);
        else if (keuze == "2") RunLeesLive(uart);
        else if (keuze == "3") { ka.Stop(); }
        else if (keuze == "4") { ka.Stop(); ka.Shutdown(); break; }
    }
    return 0;
}


// ─────────────────────────────────────────────────────────────────
//  Modus 3: Autonoom rijden + mappen
// ─────────────────────────────────────────────────────────────────

static int RunRijdenEnMappen(Pi5UARTHandler& uart, LIDAR& lidar) {
    Localisation loc(235.0f);
    Mapper       mapper(260, 160, 0.05f);

    constexpr float DT            = 0.1f;
    constexpr long  LOOP_US       = 100000;
    constexpr float LIN_SPEED     = 278.0f;
    constexpr int   HERPLAN_SCANS = 15;

    // Exploratie-eindcondities
    constexpr int   MIN_DEKKING_PCT  = 20;
    constexpr int   FRONTIER_DREMPEL = 3;
    constexpr int   MISLUKT_DREMPEL  = 5;
    constexpr float HOME_DREMPEL_MM  = 300.0f;
    constexpr float ONTSNAP_RADIUS   = 700.0f;

    // Vastzit-timeout
    constexpr int   VASTZIT_TIMEOUT  = 400;
    constexpr float VASTZIT_BEWEG_MM = 150.0f;

    // Ontwijkfase-parameters
    constexpr int VRIJRIJ_TICKS    = 25;
    constexpr int ACHTERUIT_TICKS  = 18;
    constexpr int VASTZIT_DREMPEL  = 2;

    int   scanCount         = 0;
    int   scansSindsHerplan = HERPLAN_SCANS;
    int   mislukteTeller    = 0;
    int   vastzitTicks      = 0;
    float vastzitRefX       = 0.0f;
    float vastzitRefY       = 0.0f;

    // ── IMU-offset: nul de yaw bij opstart ───────────────────────
    float imuOffset  = 0.0f;
    bool  imuGenulld = false;

    // ── Frontier blacklist ────────────────────────────────────────
    std::vector<BlacklistItem> frontierBlacklist;

    PathPlanner planner(mapper.GetMap());
    Navigator   navigator;
    bool        heeftPad = false;

    Position beginPunt(0.0f, 0.0f, 0.0f);
    bool     beginPuntVergrendeld = false;

    enum class ExploratieStaat { EXPLOREREN, TERUG_HOME, KLAAR };
    ExploratieStaat exploratieStaat = ExploratieStaat::EXPLOREREN;

    enum class OntwijkFase { NORMAAL, DRAAIEN, VRIJRIJDEN, ACHTERUIT };
    OntwijkFase ontwijkFase      = OntwijkFase::NORMAAL;
    float       doelHoek         = 0.0f;
    float       draaiRichting    = 0.0f;
    int         vrijrijTicks     = 0;
    int         achteruitTicks   = 0;
    int         vastzitTeller    = 0;
    int         achteruitEscalatie = 0;
    float       ontsnapX         = 1e9f;
    float       ontsnapY         = 1e9f;

    CommandKeepAlive ka(uart);
    ka.Start();

    float lastRanges[360] = {};
    bool  heeftRanges     = false;

    usleep(1200000);  // LIDAR spin-up

    while (!heeftRanges && running) {
        if (lidar.Update()) {
            for (int a = 0; a < 360; ++a)
                lastRanges[a] = lidar.GetDistance(a).distance;
            heeftRanges = true;
        } else {
            usleep(100000);
        }
    }

    ka.SetCommand(LIN_SPEED, 0.0f);

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        // ── 1. Sensoren ──────────────────────────────────────────
        uart.LeesData();
        SensorData sens = uart.GetSensorData();
        if (sens.geldig) {
            // Nul de IMU bij de eerste geldige meting
            if (!imuGenulld) {
                imuOffset  = sens.yawGraden;
                imuGenulld = true;
            }
            loc.Predict(sens.speedLinks, sens.speedRechts, DT);
            loc.UpdateIMU(sens.yawGraden - imuOffset, DT);
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
            for (int a = 0; a < 360; ++a) {
                lastRanges[a] = lidar.GetDistance(a).distance;
                angles[a]     = static_cast<float>(a);
            }
            mapper.Update(lastRanges, angles, 360, pos);
            heeftRanges = true;
            ++scanCount; ++scansSindsHerplan;
            nieuweScan = true;
        }

        ScanAnalyse scan{};
        if (heeftRanges) scan = AnalyseerScan(lastRanges);

        // ── 3. Vastzit-timeout ────────────────────────────────────
        if (ontwijkFase != OntwijkFase::NORMAAL &&
            exploratieStaat == ExploratieStaat::EXPLOREREN)
        {
            if (vastzitTicks == 0) { vastzitRefX = pos.GetX(); vastzitRefY = pos.GetY(); }
            ++vastzitTicks;

            float beweging = std::sqrt(std::pow(pos.GetX()-vastzitRefX, 2.0f)
                                     + std::pow(pos.GetY()-vastzitRefY, 2.0f));
            if (beweging > VASTZIT_BEWEG_MM) {
                vastzitRefX = pos.GetX(); vastzitRefY = pos.GetY();
                vastzitTicks = 0;
            }

            if (vastzitTicks >= VASTZIT_TIMEOUT) {
                exploratieStaat    = ExploratieStaat::TERUG_HOME;
                ontwijkFase        = OntwijkFase::NORMAAL;
                heeftPad           = false;
                vastzitTicks       = vastzitTeller = achteruitEscalatie = 0;
                scansSindsHerplan  = HERPLAN_SCANS;
                Path pad = planner.PlanPath(pos, beginPunt, mapper.GetMap());
                if (!pad.IsEmpty()) { navigator.SetPath(pad); heeftPad = true; }
            }
        } else {
            vastzitTicks = 0;
        }

        // ── 4. Rijlogica ──────────────────────────────────────────

        if (exploratieStaat == ExploratieStaat::KLAAR) {
            running = false;
            break;
        }

        // Obstakelontwijking — geldig in alle exploratietoestanden
        if (ontwijkFase == OntwijkFase::DRAAIEN) {
            float fout = NormDeg(doelHoek - loc.GetTheta());
            if (std::fabs(fout) < 8.0f) {
                ontwijkFase       = OntwijkFase::VRIJRIJDEN;
                int baseTicks     = (scan.minVoor < 600.0f) ? 10 : VRIJRIJ_TICKS;
                vrijrijTicks      = baseTicks + achteruitEscalatie * 8;
                scansSindsHerplan = HERPLAN_SCANS;
                ka.SetCommand(LIN_SPEED, 0.0f);
            } else if (scan.staat >= 3 &&
                       ((draaiRichting > 0 && scan.minRechts < 350.0f) ||
                        (draaiRichting < 0 && scan.minLinks  < 350.0f))) {
                draaiRichting = -draaiRichting;
                doelHoek = NormDeg(loc.GetTheta() - draaiRichting * 90.0f);
                ka.SetCommand(0.0f, draaiRichting * 40.0f);
            } else {
                ka.SetCommand(0.0f, draaiRichting * 40.0f);
            }

        } else if (ontwijkFase == OntwijkFase::VRIJRIJDEN) {
            if (scan.staat >= 3) {
                ++vastzitTeller;
                if (vastzitTeller >= VASTZIT_DREMPEL) {
                    ++achteruitEscalatie;
                    achteruitTicks = ACHTERUIT_TICKS + std::min(achteruitEscalatie * 8, 40);
                    ontwijkFase    = OntwijkFase::ACHTERUIT;
                    ka.SetCommand(-LIN_SPEED * 0.6f, 0.0f);
                } else {
                    draaiRichting = -draaiRichting;
                    doelHoek      = NormDeg(loc.GetTheta() - draaiRichting * 90.0f);
                    ontwijkFase   = OntwijkFase::DRAAIEN;
                    ka.SetCommand(0.0f, draaiRichting * 40.0f);
                }
            } else {
                if (--vrijrijTicks <= 0) {
                    vastzitTeller = achteruitEscalatie = 0;
                    ontwijkFase   = OntwijkFase::NORMAAL;
                    heeftPad      = false;
                    scansSindsHerplan = HERPLAN_SCANS;
                } else {
                    ka.SetCommand(LIN_SPEED, 0.0f);
                }
            }

        } else if (ontwijkFase == OntwijkFase::ACHTERUIT) {
            auto kiesBesteHoek = [&]() {
                float besteRuimte = 0.0f, besteHoek = 0.0f;
                for (int s = 0; s < 36; ++s) {
                    float ruimte = 8000.0f;
                    for (int a = s*10; a < s*10+10; ++a)
                        if (lastRanges[a] > 0.0f && lastRanges[a] < ruimte) ruimte = lastRanges[a];
                    if (ruimte > besteRuimte) { besteRuimte = ruimte; besteHoek = (float)(s*10); }
                }
                if (besteHoek > 180.0f) besteHoek -= 360.0f;
                return besteHoek;
            };

            if (scan.minAchter < 300.0f || --achteruitTicks <= 0) {
                float bh  = kiesBesteHoek();
                doelHoek  = NormDeg(loc.GetTheta() + bh);
                draaiRichting = (std::fabs(bh) < 5.0f) ? 1.0f : (bh >= 0.0f ? 1.0f : -1.0f);
                ontwijkFase   = OntwijkFase::DRAAIEN;
                ka.SetCommand(0.0f, draaiRichting * 40.0f);
                ontsnapX = loc.GetX(); ontsnapY = loc.GetY();
            } else {
                ka.SetCommand(-LIN_SPEED * 0.6f, 0.0f);
            }

        } else if (exploratieStaat == ExploratieStaat::TERUG_HOME) {
            float dx = pos.GetX() - beginPunt.GetX();
            float dy = pos.GetY() - beginPunt.GetY();

            if (std::sqrt(dx*dx+dy*dy) < HOME_DREMPEL_MM) {
                ka.Stop();
                exploratieStaat = ExploratieStaat::KLAAR;
                mapper.SaveDebugMap("kaart.pgm");
            } else {
                if (!heeftPad || navigator.IsFinished()) {
                    heeftPad = false;
                    Path pad = planner.PlanPath(pos, beginPunt, mapper.GetMap());
                    if (!pad.IsEmpty()) { navigator.SetPath(pad); heeftPad = true; }
                    else {
                        float hoek = std::atan2(-dy,-dx)*(180.0f/static_cast<float>(M_PI));
                        ka.SetCommand(LIN_SPEED*0.6f, NormDeg(hoek-loc.GetTheta())*0.5f);
                    }
                }
                if (heeftPad && !navigator.IsFinished()) {
                    navigator.Update(pos);
                    DriveCommand cmd = navigator.GetNextCommand(pos);
                    float extraAng = (scan.staat == 1) ? scan.uitwijkHoek * 8.0f : 0.0f;
                    ka.SetCommand(cmd.GetLinVelocity(), cmd.GetAngVelocity() + extraAng);
                }
                if (scan.staat >= 3) {
                    ontsnapX = loc.GetX(); ontsnapY = loc.GetY();
                    draaiRichting = scan.uitwijkHoek;
                    doelHoek      = NormDeg(loc.GetTheta() - draaiRichting * 90.0f);
                    ontwijkFase   = OntwijkFase::DRAAIEN;
                    heeftPad      = false;
                    ka.SetCommand(0.0f, draaiRichting * 40.0f);
                }
            }

        } else {
            // ── EXPLOREREN ────────────────────────────────────────
            if (scan.staat >= 3) {
                ontsnapX = loc.GetX(); ontsnapY = loc.GetY();
                draaiRichting = scan.uitwijkHoek;
                doelHoek      = NormDeg(loc.GetTheta() - draaiRichting * 90.0f);
                ontwijkFase   = OntwijkFase::DRAAIEN;
                heeftPad      = false;
                ka.SetCommand(0.0f, draaiRichting * 40.0f);

            } else if (scan.staat == 2) {
                ka.SetCommand(LIN_SPEED * 0.35f, scan.uitwijkHoek * 22.0f);

            } else if (heeftPad && !navigator.IsFinished()) {
                navigator.Update(pos);
                DriveCommand cmd = navigator.GetNextCommand(pos);

                // ── Gangcentrering ────────────────────────────────
                // Als beide zijmuren < 1200mm: stuur zacht bij op
                // het verschil zodat de robot gecentreerd blijft.
                // Alleen actief als scan.staat == 0 (geen obstakel).
                float gangCorr = 0.0f;
                if (scan.staat == 0 &&
                    scan.ruimteLinks  < 1200.0f &&
                    scan.ruimteRechts < 1200.0f)
                {
                    float verschil = scan.ruimteRechts - scan.ruimteLinks;
                    gangCorr = verschil * 0.015f;
                    if (gangCorr >  8.0f) gangCorr =  8.0f;
                    if (gangCorr < -8.0f) gangCorr = -8.0f;
                }

                float extraAng = (scan.staat == 1) ? scan.uitwijkHoek * 8.0f : 0.0f;
                ka.SetCommand(cmd.GetLinVelocity(),
                              cmd.GetAngVelocity() + extraAng + gangCorr);

            } else {
                if (heeftPad && navigator.IsFinished()) {
                    heeftPad = false;
                    scansSindsHerplan = HERPLAN_SCANS;
                }

                if (nieuweScan && scansSindsHerplan >= HERPLAN_SCANS) {
                    scansSindsHerplan = 0;

                    int  aantalFrontiers = TelFrontiers(mapper);
                    int  dekking         = mapper.GetCoverage();
                    bool dekkingOk       = (dekking >= MIN_DEKKING_PCT);

                    if (dekkingOk && (aantalFrontiers <= FRONTIER_DREMPEL ||
                                      mislukteTeller  >= MISLUKT_DREMPEL)) {
                        exploratieStaat   = ExploratieStaat::TERUG_HOME;
                        heeftPad          = false;
                        mislukteTeller    = 0;
                        scansSindsHerplan = HERPLAN_SCANS;
                        Path pad = planner.PlanPath(pos, beginPunt, mapper.GetMap());
                        if (!pad.IsEmpty()) { navigator.SetPath(pad); heeftPad = true; }
                    } else {
                        TickBlacklist(frontierBlacklist);
                        Position doel = KiesFrontierDoel(mapper, pos, lastRanges,
                                                         frontierBlacklist);

                        // Sla frontiers over die te dicht bij de ontsnappositie liggen
                        float ddx = doel.GetX()-ontsnapX, ddy = doel.GetY()-ontsnapY;
                        if ((ddx*ddx+ddy*ddy) < (ONTSNAP_RADIUS*ONTSNAP_RADIUS) && ontsnapX < 1e8f) {
                            Position nepPos(ontsnapX, ontsnapY, pos.GetTheta());
                            Position alt = KiesFrontierDoel(mapper, nepPos, lastRanges,
                                                            frontierBlacklist);
                            float dx2=alt.GetX()-ontsnapX, dy2=alt.GetY()-ontsnapY;
                            if (dx2*dx2+dy2*dy2 > ddx*ddx+ddy*ddy) doel = alt;
                            else ontsnapX = ontsnapY = 1e9f;
                        }

                        if (doel.GetX() == pos.GetX() && doel.GetY() == pos.GetY()) {
                            ++mislukteTeller;
                            // Geen blacklist — er was geen doel gevonden
                            ka.SetCommand(LIN_SPEED, 0.0f);
                            scansSindsHerplan = HERPLAN_SCANS;
                        } else {
                            Path pad = planner.PlanPath(pos, doel, mapper.GetMap());
                            if (!pad.IsEmpty()) {
                                navigator.SetPath(pad);
                                heeftPad = true;
                                mislukteTeller = 0;
                            } else {
                                ++mislukteTeller;
                                VoegToeAanBlacklist(frontierBlacklist,
                                                    doel.GetX(), doel.GetY());
                                ka.SetCommand(LIN_SPEED, 0.0f);
                                scansSindsHerplan = HERPLAN_SCANS;
                            }
                        }
                    }
                } else {
                    float extraAng = (scan.staat == 1) ? scan.uitwijkHoek * 8.0f : 0.0f;
                    ka.SetCommand(LIN_SPEED, extraAng);
                }
            }
        }

        // ── 5. Loop timing ────────────────────────────────────────
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::steady_clock::now() - tStart).count();
        long rest = LOOP_US - elapsed;
        if (rest > 0) usleep(static_cast<useconds_t>(rest));
    }

    ka.Stop(); ka.Shutdown();
    for (int i = 0; i < 10; ++i) { uart.StuurStop(); usleep(50000); }
    lidar.Disconnect();
    mapper.SaveDebugMap("kaart.pgm");
    return 0;
}


// ─────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────

enum class MenuKeuze { MAPPEN, PICO_COMMUNICATIE, RIJDEN_EN_MAPPEN, STOPPEN };

static MenuKeuze VraagMenuKeuze() {
    while (true) {
        std::cout << "\n"
                  << "╔══════════════════════════════════════╗\n"
                  << "║        ROBOT CONTROLLER  Pi5         ║\n"
                  << "╠══════════════════════════════════════╣\n"
                  << "║  1. Mappen                           ║\n"
                  << "║  2. Pico communiceren                ║\n"
                  << "║  3. Autonoom rijden + mappen         ║\n"
                  << "║  4. Stoppen                          ║\n"
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
    LIDAR          lidar("/dev/ttyUSB0", 460800);

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