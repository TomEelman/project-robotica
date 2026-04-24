#pragma once

#include <string>
#include <vector>

struct SensorData {
    std::string        type;
    std::vector<float> waarden;
};

// UART 
int         openSerial   (const char* port);
std::string vraag_sensor (int fd, const std::string& sensor);
// Parser 
SensorData  parseResponse  (const std::string& response);
void        printSensorData(const SensorData& data);

// LIDAR scan
void doLidarScan(const std::string& port, int baudrate);