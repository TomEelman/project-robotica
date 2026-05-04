#pragma once

#include <string>
#include <vector>

// ── Sensor data structuur ─────────────────────────────────────────
struct SensorData {
    std::string        type;
    std::vector<float> waarden;
};

// ── UART ─────────────────────────────────────────────────────────
int         openSerial   (const char* port);
std::string vraag_sensor (int fd, const std::string& sensor);

// ── Parser ────────────────────────────────────────────────────────
SensorData  parseResponse  (const std::string& response);
void        printSensorData(const SensorData& data);

// ── LIDAR-only scan (menu optie 2) ───────────────────────────────
void doLidarScan(const std::string& port, int baudrate);

// ── Live mapping met EKF + GridMap (menu optie 3) ─────────────────
void doLiveMapping(const std::string& lidarPort,  int lidarBaud,
                   int serialFd);