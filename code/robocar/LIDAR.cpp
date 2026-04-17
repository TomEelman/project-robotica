#include "LIDAR.h"
#include <iostream>
#include <cstring>
#include <unistd.h>

using namespace sl;

LIDAR::LIDAR(const std::string& port, int baudRate)
    : port(port),
      baudRate(baudRate),
      maxRange(4000.0f),
      minRange(0.0f),
      channel(nullptr),
      driver(nullptr)
{
    for (int i = 0; i < SCAN_SIZE; i++) {
        scanData[i].angle    = i;
        scanData[i].distance = 0.0f;
    }
}

LIDAR::~LIDAR() {
    Disconnect();
}

bool LIDAR::Connect() {
    auto result = createLidarDriver();
    if (!result) {
        std::cerr << "LIDAR: failed to create driver\n";
        return false;
    }
    driver = *result;

    auto chanResult = createSerialPortChannel(port, baudRate);
    if (!chanResult) {
        std::cerr << "LIDAR: failed to create serial channel\n";
        delete driver;
        driver = nullptr;
        return false;
    }
    channel = *chanResult;

    if (!SL_IS_OK(driver->connect(channel))) {
        std::cerr << "LIDAR: failed to connect to " << port << "\n";
        delete driver;
        driver = nullptr;
        return false;
    }

    sl_lidar_response_device_info_t devInfo;
    if (!SL_IS_OK(driver->getDeviceInfo(devInfo))) {
        std::cerr << "LIDAR: failed to get device info\n";
        Disconnect();
        return false;
    }

    driver->setMotorSpeed();
    driver->startScan(0, 1);
    usleep(2000000); // 2s for motor spin-up

    return true;
}

void LIDAR::Disconnect() {
    if (driver) {
        driver->stop();
        driver->setMotorSpeed(0);
        delete driver;
        driver = nullptr;
    }
    channel = nullptr;
}

bool LIDAR::Update() {
    if (!driver) return false;

    // Reset all distances, keep angles intact
    for (int i = 0; i < SCAN_SIZE; i++) {
        scanData[i].distance = 0.0f;
    }

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    sl_result op = driver->grabScanDataHq(nodes, count);
    if (!SL_IS_OK(op)) {
        std::cerr << "LIDAR: grabScanDataHq failed: " << op << "\n";
        return false;
    }

    driver->ascendScanData(nodes, count);

    for (size_t i = 0; i < count; i++) {
        float angle    = (nodes[i].angle_z_q14 * 90.f) / 16384.f;
        float distance = static_cast<float>(nodes[i].dist_mm_q2) / 4.0f;
        int   quality  = nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

        int idx = static_cast<int>(angle) % SCAN_SIZE;

        if (quality > 0 && distance > minRange && distance < maxRange) {
            scanData[idx].angle    = idx;
            scanData[idx].distance = distance;
        }
    }

    return true;
}

LIDAR::ScanEntry LIDAR::GetDistance(int angle) const {
    if (angle < 0 || angle >= SCAN_SIZE) return { angle, 0.0f };
    return scanData[angle];
}

bool LIDAR::IsObjectInRange(int minAngle, int maxAngle, float threshold) const {
    for (int a = minAngle; a <= maxAngle; a++) {
        float d = scanData[a % SCAN_SIZE].distance;
        if (d > minRange && d < threshold) return true;
    }
    return false;
}

void LIDAR::ApplyMotionCorrection(float currentYaw) {
    int shift = static_cast<int>(currentYaw) % SCAN_SIZE;
    if (shift == 0) return;

    ScanEntry tmp[SCAN_SIZE];
    memcpy(tmp, scanData, sizeof(scanData));
    for (int i = 0; i < SCAN_SIZE; i++) {
        int newIdx = (i + shift + SCAN_SIZE) % SCAN_SIZE;
        scanData[newIdx]       = tmp[i];
        scanData[newIdx].angle = newIdx; // keep angle consistent with index
    }
}