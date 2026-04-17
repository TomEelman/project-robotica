#include "LIDAR.h"
#include <iostream>
#include <cstring>
#include <cmath>
#include <unistd.h>

using namespace sl;

LIDAR::LIDAR(const std::string& port, int baudRate)
    : port(port),
      baudRate(baudRate),
      maxRange(4000),
      minRange(0),
      channel(nullptr),
      drv(nullptr)
{
    memset(scanData, 0, sizeof(scanData));
}

LIDAR::~LIDAR() {
    Disconnect();
}

bool LIDAR::Connect() {
    // Create driver instance
    auto result = createLidarDriver();
    if (!result) {
        std::cerr << "LIDAR: failed to create driver\n";
        return false;
    }
    drv = *result;

    // Create serial channel
    auto chanResult = createSerialPortChannel(port, baudRate);
    if (!chanResult) {
        std::cerr << "LIDAR: failed to create serial channel\n";
        delete drv;
        drv = nullptr;
        return false;
    }
    channel = *chanResult;

    // Connect
    if (!SL_IS_OK(drv->connect(channel))) {
        std::cerr << "LIDAR: failed to connect to " << port << "\n";
        delete drv;
        drv = nullptr;
        return false;
    }

    // Verify device is responsive
    sl_lidar_response_device_info_t devInfo;
    if (!SL_IS_OK(drv->getDeviceInfo(devInfo))) {
        std::cerr << "LIDAR: failed to get device info\n";
        Disconnect();
        return false;
    }

    std::cout << "LIDAR: connected.";
    drv->setMotorSpeed();
    drv->startScan(0, 1);
    usleep(2000000);

    return true;
}

void LIDAR::Disconnect() {
    if (drv) {
        drv->stop();
        drv->setMotorSpeed(0);
        delete drv;
        drv = nullptr;
    }
    // channel is owned by the driver, no separate delete needed
    channel = nullptr;
}

bool LIDAR::Update() {
    if (!drv) return false;

    memset(scanData, 0, sizeof(scanData));

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    sl_result op = drv->grabScanDataHq(nodes, count);
    if (!SL_IS_OK(op)) {
        std::cerr << "LIDAR: grabScanDataHq failed: " << op << "\n";
        return false;
    }

    drv->ascendScanData(nodes, count);

    for (size_t i = 0; i < count; i++) {
        float angle    = (nodes[i].angle_z_q14 * 90.f) / 16384.f;
        float distance = static_cast<float>(nodes[i].dist_mm_q2) / 4.0f;
        int   quality  = nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

        int angleIdx = static_cast<int>(angle) % SCAN_SIZE;

        if (quality > 0 && distance > static_cast<float>(minRange)
                        && distance < static_cast<float>(maxRange)) {
            scanData[angleIdx] = static_cast<int>(distance);
        }
    }

    return true;
}
int LIDAR::GetDistance(int angle) const {
    if (angle < 0 || angle >= SCAN_SIZE) return 0;
    return scanData[angle];
}

bool LIDAR::IsObjectInRange(int minAngle, int maxAngle, int threshold) const {
    for (int a = minAngle; a <= maxAngle; a++) {
        int d = scanData[a % SCAN_SIZE];
        if (d > minRange && d < threshold) return true;
    }
    return false;
}

void LIDAR::ApplyMotionCorrection(float currentYaw) {
    int shift = static_cast<int>(currentYaw) % SCAN_SIZE;
    if (shift == 0) return;

    int tmp[SCAN_SIZE];
    memcpy(tmp, scanData, sizeof(scanData));
    for (int i = 0; i < SCAN_SIZE; i++) {
        scanData[(i + shift + SCAN_SIZE) % SCAN_SIZE] = tmp[i];
    }
}