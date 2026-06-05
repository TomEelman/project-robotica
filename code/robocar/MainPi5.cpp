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
    uart.SendStop();
}

void CommandKeepAlive::Start() {
    threadActief = true;
    worker = std::thread([this]() {
        while (threadActief) {
            { std::lock_guard<std::mutex> lk(mtx);
                if (actief && running) uart.SendCommand(lin, ang);
                else if (!running) { uart.SendStop(); actief = false; }
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

static void EncoderWithSign(float cmdLin, float cmdAng, float& vLeft, float& vRight) {
    constexpr float HALF_BASE = 219.0f / 2.0f;
    constexpr float DEG2RAD   = static_cast<float>(M_PI) / 180.0f;

    float omegaRad  = cmdAng * DEG2RAD;
    float cmdVLeft  = cmdLin + omegaRad * HALF_BASE;
    float cmdVRight = cmdLin - omegaRad * HALF_BASE;

    float signLeft  = (cmdVLeft  >= 0.0f) ? 1.0f : -1.0f;
    float signRight = (cmdVRight >= 0.0f) ? 1.0f : -1.0f;

    vLeft  = signLeft  * std::fabs(vLeft);
    vRight = signRight * std::fabs(vRight);
}

static int RunPathFinding(Pi5UARTHandler& uart, LIDAR& lidar) {
    Localisation loc(219.0f);
    Mapper mapper(260, 160, 0.03f);

    constexpr float DT          = 0.1f;
    constexpr long  LOOP_US     = 100000;
    constexpr float LIN_SPEED   = 278.0f;
    constexpr int   REPLAN_SCANS   = 15;
    constexpr float HOME_THRESHOLD_MM = 300.0f;
    constexpr float lidarScanTime     = 0.1f;

    NavMode_HEAD navMode = NavMode_HEAD::FRONTIER;

    int scanCount         = 0;
    int scansSinceReplan = REPLAN_SCANS;

    constexpr int   STUCK_TIMEOUT  = 400;
    constexpr float STUCK_MOVE_MM = 150.0f;

    int   stuckTicks = 0;
    float stuckRefX  = 0.0f;
    float stuckRefY = 0.0f;

    float imuOffset  = 0.0f;
    bool  imuNulled = false;
    //float currentImuYaw = 0.0f;
    float omegaDegS     = 0.0f;

    ExplorationPlanner         explorationPlanner;
    std::vector<BlacklistItem> frontierBlacklist;
    PathPlanner planner(mapper.GetMap(), true);
    Navigator   navigator;
    bool        hasPath = false;

    Position StartPoint(0.0f, 0.0f, 0.0f);
    bool     startPointLocked = false;

    NavMode_EVADING evadeMode = NavMode_EVADING::NORMAL;
    float goalAngle = 0.0f;
    float currentAngle = 0.0f;
    int clearDriveTicks = 0;
    int reverseTicks = 0;
    int stuckCounter = 0;
    int reverseEscalation = 0;
    float escapeX = 1e9f;
    float escapeY = 1e9f;

    CommandKeepAlive ka(uart);
    ScanMatcher scanMatcher;
    //float linSpeed = 0.0f;

    float lastScanX = 0.0f;
    float lastScanY = 0.0f;
    bool  hasLastScanPos = false;

    float lastCommandSign = 0.0f;  
    bool  waitRollout = false;
    float lastRanges[360] = {};
    bool hasRanges = false;

    usleep(1200000);
    for (int i = 0; i < 5; ++i) { uart.SendStop(); usleep(20000); }
    tcflush(uart.GetFd(), TCIFLUSH);
    for (int i = 0; i < 15; ++i) { uart.ReadData(); usleep(10000); }

    while (!hasRanges && running) {
        if (lidar.Update()) {
            for (int a = 0; a < 360; ++a) lastRanges[a] = lidar.GetDistance(a).distance;
            hasRanges = true;
        } else usleep(100000);
    }

    ka.Start();

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        bool newData = uart.ReadData();  
        SensorData sens = uart.GetSensorData();
        if (sens.valid) {
            if (!imuNulled) { imuOffset = sens.yawDegrees; imuNulled = true; }

            float vL = sens.speedLeft, vR = sens.speedRight;
            EncoderWithSign(ka.GetLin(), ka.GetAng(), vL, vR);

            omegaDegS = ka.GetAng();
            //linSpeed  = 0.5f * (vL + vR);  

            float cmdLinNow  = ka.GetLin();
            float cmdSignNow = (cmdLinNow > 1.0f) ? 1.0f : (cmdLinNow < -1.0f) ? -1.0f : 0.0f;

            bool signFlipped = (lastCommandSign > 0.0f && cmdSignNow < 0.0f) || (lastCommandSign < 0.0f && cmdSignNow > 0.0f);
            if (signFlipped) {
                waitRollout = true;
            }
            lastCommandSign = cmdSignNow;

            constexpr float ROLLOUT_DONE_MM_S = 15.0f;
            if (waitRollout &&
                std::fabs(sens.speedLeft)  < ROLLOUT_DONE_MM_S &&
                std::fabs(sens.speedRight) < ROLLOUT_DONE_MM_S) {
                waitRollout = false;
            }

            if (newData && !waitRollout) {
                loc.Predict(vL, vR, DT);
                loc.UpdateIMU(sens.yawDegrees - imuOffset, DT);
            } else if (newData && waitRollout) {
                loc.UpdateIMU(sens.yawDegrees - imuOffset, DT);
            }

            //currentImuYaw = sens.yawDegrees - imuOffset;

            if (!startPointLocked) {
                StartPoint = Position(loc.GetX(), loc.GetY(), loc.GetTheta());
                startPointLocked = true;
            }
        }

        Position pos(loc.GetX(), loc.GetY(), loc.GetTheta());

        //bool newScan = false;
        if (lidar.Update()) {
            float angles[360];
            for (int a = 0; a < 360; ++a) {
                lastRanges[a] = lidar.GetDistance(a).distance;
                angles[a]     = static_cast<float>(a);
            }

            float encDx = 0.0f, encDy = 0.0f;
            if (hasLastScanPos) {
                encDx = loc.GetX() - lastScanX;
                encDy = loc.GetY() - lastScanY;
            }
            scanMatcher.Match(lastRanges, encDx, encDy);

            loc.SetIcpAnchor(); 

            lastScanX      = loc.GetX();
            lastScanY      = loc.GetY();
            hasLastScanPos = true;

            mapper.UpdateMotionCorrected(lastRanges, angles, 360, pos, omegaDegS, lidarScanTime);
            hasRanges= true;
            ++scanCount;
            ++scansSinceReplan;
            //newScan = true;
        }

        ScanAnalysis scan{};
        if (hasRanges) scan = AnalyzeScan(lastRanges);

        if (evadeMode != NavMode_EVADING::NORMAL && navMode != NavMode_HEAD::RETURN_HOME) {
            if (stuckTicks == 0) { 
                stuckRefX = pos.GetX(); 
                stuckRefY = pos.GetY(); 
            }
            ++stuckTicks;

            float movement = std::hypot(pos.GetX() - stuckRefX, pos.GetY() - stuckRefY);

            if (movement > STUCK_MOVE_MM) {
                stuckRefX = pos.GetX(); stuckRefY = pos.GetY(); stuckTicks = 0;
            }

            if (stuckTicks >= STUCK_TIMEOUT) {
                navMode = NavMode_HEAD::RETURN_HOME;
                evadeMode = NavMode_EVADING::NORMAL;
                hasPath = false;
                scansSinceReplan = REPLAN_SCANS;
                Path path = planner.PlanPath(pos, StartPoint, mapper.GetMap());
                if (!path.IsEmpty()) { navigator.SetPath(path); mapper.SetWaypoints(path); hasPath = true; }
            }
        } else {
            stuckTicks = 0;
        }

        if (evadeMode != NavMode_EVADING::NORMAL) {
            navigator.HandleObstacleAvoidance(scan, loc, evadeMode, goalAngle, currentAngle,
                clearDriveTicks, reverseTicks, stuckCounter, reverseEscalation,
                escapeX, escapeY, ka, LIN_SPEED, scansSinceReplan);
        } else {
            switch (navMode) {
                case NavMode_HEAD::FRONTIER:
                    navigator.HandleFrontierMode(mapper, pos, newData, scansSinceReplan,
                        navMode, hasPath, ka, planner,
                        frontierBlacklist, REPLAN_SCANS,
                        lastRanges, scan.minFront);
                    break;
                case NavMode_HEAD::RETURN_HOME:
                    navigator.HandleReturnToHome(mapper, pos, StartPoint, hasPath, ka, planner, HOME_THRESHOLD_MM, navMode);
                    break;
                case NavMode_HEAD::DONE:
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
    for (int i = 0; i < 10; ++i) { uart.SendStop(); usleep(50000); }
    lidar.Disconnect();
    return 0;
}

static int RunWallFollowing(Pi5UARTHandler& uart, LIDAR& lidar) {
    Localisation loc(219.0f);
    Mapper mapper(260, 160, 0.03f);

    constexpr float DT          = 0.1f;
    constexpr long  LOOP_US     = 100000;
    constexpr float lidarScanTime = 0.1f;

    int   scanCount     = 0;
    float imuOffset     = 0.0f;
    bool  imuNulled    = false;
    //float currentImuYaw = 0.0f;
    float omegaDegS     = 0.0f;

    Navigator   navigator;
    navigator.ResetWallFollower();

    CommandKeepAlive ka(uart);
    ScanMatcher      scanMatcher; 

    //float linSpeed       = 0.0f; 

    float lastScanX      = 0.0f, lastScanY = 0.0f;
    bool  hasLastScanPos = false;

    float lastRanges[360] = {};
    bool  hasRanges     = false;

    usleep(1200000);
    for (int i = 0; i < 5; ++i) { uart.SendStop(); usleep(20000); }
    tcflush(uart.GetFd(), TCIFLUSH);
    for (int i = 0; i < 15; ++i) { uart.ReadData(); usleep(10000); }

    while (!hasRanges && running) {
        if (lidar.Update()) {
            for (int a = 0; a < 360; ++a) lastRanges[a] = lidar.GetDistance(a).distance;
            hasRanges = true;
        } else usleep(100000);
    }

    ka.Start();

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        bool newData = uart.ReadData();   
        SensorData sens = uart.GetSensorData();
        if (sens.valid) {
            if (!imuNulled) { imuOffset = sens.yawDegrees; imuNulled = true; }

            float vL = sens.speedLeft, vR = sens.speedRight;
            EncoderWithSign(ka.GetLin(), ka.GetAng(), vL, vR);

            omegaDegS = ka.GetAng();
            //linSpeed  = 0.5f * (vL + vR);

            bool isMoving = (sens.speedLeft != 0.0f || sens.speedRight != 0.0f);

            if (newData && isMoving) {
                loc.Predict(vL, vR, DT);
                loc.UpdateIMU(sens.yawDegrees - imuOffset, DT);
            }

            //currentImuYaw = sens.yawDegrees - imuOffset;

            if (!newData && isMoving) {
                //printf("[UART] geen vers pakket deze tick — Predict overgeslagen\n");
            }
        }

        Position pos(loc.GetX(), loc.GetY(), loc.GetTheta());

        if (lidar.Update()) {
            float angles[360];
            for (int a = 0; a < 360; ++a) {
                lastRanges[a] = lidar.GetDistance(a).distance;
                angles[a]     = static_cast<float>(a);
            }

            float encDx = 0.0f, encDy = 0.0f;
            if (hasLastScanPos) {
                encDx = loc.GetX() - lastScanX;
                encDy = loc.GetY() - lastScanY;
            }
            
            scanMatcher.Match(lastRanges, encDx, encDy);

            loc.SetIcpAnchor();

            lastScanX      = loc.GetX();
            lastScanY      = loc.GetY();
            hasLastScanPos = true;

            mapper.UpdateMotionCorrected(lastRanges, angles, 360, pos, omegaDegS, lidarScanTime);
            hasRanges = true;
            ++scanCount;
        }

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
    for (int i = 0; i < 10; ++i) { uart.SendStop(); usleep(50000); }
    lidar.Disconnect();
    return 0;
}

static int RunMapping(Pi5UARTHandler& uart, LIDAR& lidar) {
    Localisation loc(219.0f); 
    Mapper mapper(260, 160, 0.03f);
    constexpr float DT = 0.1f, LOOP_US = 100000;

    float imuOffset  = 0.0f;
    bool  imuNulled = false;
    int   scanCount  = 0;

    usleep(1200000);

    while (running) {
        auto tStart = std::chrono::steady_clock::now();

        uart.ReadData();
        SensorData sens = uart.GetSensorData();
        if (sens.valid) {
            if (!imuNulled) { imuOffset = sens.yawDegrees; imuNulled = true; }
            loc.Predict(sens.speedLeft, sens.speedRight, DT);
            loc.UpdateIMU(sens.yawDegrees - imuOffset, DT);
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

        // mapper.PrintMap(loc.GetX(), loc.GetY(), scanCount, mapper.GetCoverage());

        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - tStart).count();
        long rest = (long)LOOP_US - elapsed;
        if (rest > 0) usleep((useconds_t)rest);
    }

    lidar.Disconnect();
    mapper.SaveDebugMap("kaart.pgm");
    return 0;
}

static void SetTerminalRaw(bool raw) {
    static struct termios orig;
    static bool saved = false;
    if (!saved) { tcgetattr(STDIN_FILENO, &orig); saved = true; }
    if (raw) {
        struct termios t = orig;
        t.c_lflag &= ~(ICANON | ECHO);
        t.c_cc[VMIN]  = 0;   
        t.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
    } else {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig);
        tcflush(STDIN_FILENO, TCIFLUSH);  
    }
}

static int PollKey() {
    char c;
    return (read(STDIN_FILENO, &c, 1) == 1) ? static_cast<int>((unsigned char)c) : -1;
}

static int RunPicoCommunication(Pi5UARTHandler& uart, LIDAR& lidar) {
    constexpr float DT          = 0.1f;
    constexpr long  LOOP_US     = 100000;
    constexpr float lidarScanTime = 0.1f;

    Localisation loc(219.0f);
    Mapper       mapper(260, 160, 0.03f);
    ScanMatcher  scanMatcher; 

    float imuOffset     = 0.0f;
    bool  imuNulled    = false;
    //float currentImuYaw = 0.0f;
    float omegaDegS     = 0.0f;
    int   scanCount     = 0;

    float lastRanges[360] = {};
    bool  hasRanges     = false;

    float lastScanX      = 0.0f, lastScanY = 0.0f;
    bool  hasLastScanPos = false;

    CommandKeepAlive ka(uart);

    usleep(1200000);
    for (int i = 0; i < 5; ++i) { uart.SendStop(); usleep(20000); }
    tcflush(uart.GetFd(), TCIFLUSH);
    for (int i = 0; i < 15; ++i) { uart.ReadData(); usleep(10000); }

    while (!hasRanges && running) {
        if (lidar.Update()) {
            for (int a = 0; a < 360; ++a) lastRanges[a] = lidar.GetDistance(a).distance;
            hasRanges = true;
        } else usleep(100000);
    }

    ka.Start();

    printf("\n[PICO] Interactieve rijmodus\n");
    printf("  Commando's: vooruit  achteruit  links  rechts  bochtl  bochtr  stop\n");
    printf("  SPATIE = nieuw commando invoeren  |  q = stoppen en terug naar menu\n\n");

    bool stopRequested = false;

    while (running && !stopRequested) {
        SetTerminalRaw(false);
        ka.SetCommand(0.0f, 0.0f);

        printf("commando> ");
        fflush(stdout);

        std::string input;
        if (!std::getline(std::cin, input)) break;

        size_t b = input.find_first_not_of(" \t");
        if (b == std::string::npos) continue;
        size_t e = input.find_first_of(" \t", b);
        std::string name = input.substr(b, e == std::string::npos ? e : e - b);

        if (name == "q" || name == "Q") break;

        float driveLin, driveAng;
        if      (name == "vooruit")   { driveLin =  278.0f; driveAng =   0.0f; }
        else if (name == "achteruit") { driveLin = -278.0f; driveAng =   0.0f; }
        else if (name == "links")     { driveLin =    0.0f; driveAng = -40.0f; }
        else if (name == "rechts")    { driveLin =    0.0f; driveAng =  40.0f; }
        else if (name == "bochtl")    { driveLin =   20.0f; driveAng = -40.0f; }
        else if (name == "bochtr")    { driveLin =   20.0f; driveAng =  40.0f; }
        else if (name == "stop")      { driveLin =    0.0f; driveAng =   0.0f; }
        else {
            printf("[PICO] Onbekend commando '%s'\n", name.c_str());
            continue;
        }

        /*printf("  → %s  (lin=%.0f ang=%.0f)  |  SPATIE = ander commando  |  q = stoppen\n",
               name.c_str(), driveLin, driveAng);
*/
        ka.SetCommand(driveLin, driveAng);

        SetTerminalRaw(true);

        while (running && !stopRequested) {
            auto tStart = std::chrono::steady_clock::now();

            uart.ReadData();
            SensorData sens = uart.GetSensorData();
            bool  isMoving      = false;
            //float linSpeedPico = 0.0f;
            if (sens.valid) {
                if (!imuNulled) { imuOffset = sens.yawDegrees; imuNulled = true; }

                float vL = sens.speedLeft, vR = sens.speedRight;
                EncoderWithSign(ka.GetLin(), ka.GetAng(), vL, vR);

                omegaDegS      = ka.GetAng();

                isMoving = (sens.speedLeft != 0.0f || sens.speedRight != 0.0f);
                if (isMoving) {
                    loc.Predict(vL, vR, DT);
                    loc.UpdateIMU(sens.yawDegrees - imuOffset, DT);
                }
                //currentImuYaw = sens.yawDegrees - imuOffset;
            }

            if (lidar.Update()) {
                float angles[360];
                for (int a = 0; a < 360; ++a) {
                    lastRanges[a] = lidar.GetDistance(a).distance;
                    angles[a]     = static_cast<float>(a);
                }
                if (isMoving) {
                    Position pos(loc.GetX(), loc.GetY(), loc.GetTheta());
                    float encDx = 0.0f, encDy = 0.0f;
                    if (hasLastScanPos) {
                        encDx = loc.GetX() - lastScanX;
                        encDy = loc.GetY() - lastScanY;
                    }
                    scanMatcher.Match(lastRanges, encDx, encDy);
                    
                    loc.SetIcpAnchor();
                    lastScanX = loc.GetX(); lastScanY = loc.GetY(); hasLastScanPos = true;
                    mapper.UpdateMotionCorrected(lastRanges, angles, 360, pos, omegaDegS, lidarScanTime);
                    ++scanCount;
                    //mapper.PrintMap(loc.GetX(), loc.GetY(), scanCount, mapper.GetCoverage());
                }
            }

            int ch = PollKey();
            if (ch == ' ') {
                ka.SetCommand(0.0f, 0.0f);
                printf("\n");
                break;
            }
            if (ch == 'q' || ch == 'Q') {
                ka.SetCommand(0.0f, 0.0f);
                stopRequested = true;
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
    for (int i = 0; i < 10; ++i) { uart.SendStop(); usleep(50000); }
    lidar.Disconnect();
    mapper.SaveDebugMap("kaart.pgm");
    //printf("[PICO] Kaart opgeslagen als kaart.pgm\n");
    return 0;
}

enum class MenuChoice { MAPPING, PICO_COMMUNICATION, PATHFINDING, DRIVE_AND_MAP };

static MenuChoice AskMenuChoice() {
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
        std::string input; std::getline(std::cin, input);
        if (input == "1") return MenuChoice::MAPPING;
        if (input == "2") return MenuChoice::PICO_COMMUNICATION;
        if (input == "3") return MenuChoice::PATHFINDING;
        if (input == "4") return MenuChoice::DRIVE_AND_MAP;
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

    switch (AskMenuChoice()) {
        case MenuChoice::MAPPING:
            if (!lidar.Connect()) { std::cerr << "LIDAR niet beschikbaar.\n"; return 1; }
            return RunMapping(uart, lidar);
        case MenuChoice::PICO_COMMUNICATION:
            if (!lidar.Connect()) { std::cerr << "LIDAR niet beschikbaar.\n"; return 1; }
            return RunPicoCommunication(uart, lidar);
        case MenuChoice::PATHFINDING:
            if (!lidar.Connect()) { std::cerr << "LIDAR niet beschikbaar.\n"; return 1; }
            return RunPathFinding(uart, lidar);
        case MenuChoice::DRIVE_AND_MAP:
            if (!lidar.Connect()) { std::cerr << "LIDAR niet beschikbaar.\n"; return 1; }
            return RunWallFollowing(uart, lidar);
    }
    return 0;
}