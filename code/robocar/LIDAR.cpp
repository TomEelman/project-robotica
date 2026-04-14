#include "LIDAR.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <iostream>
#include <cstdio>

static constexpr uint8_t CMD_SYNC  = 0xA5;
static constexpr uint8_t CMD_SCAN  = 0x20;
static constexpr uint8_t CMD_STOP  = 0x25;
static constexpr uint8_t CMD_RESET = 0x40;
static constexpr uint8_t ANS_SYNC1 = 0xA5;
static constexpr uint8_t ANS_SYNC2 = 0x5A;

LIDAR::LIDAR(const std::string& port, int baudRate)
    : port(port),
      fd(-1),
      baudRate(baudRate),
      maxRange(4000),
      minRange(0),
      rotationSpeed(0),
      currentAngle(0),
      nextAngle(0),
      updated(false)
{
    memset(scanData, 0, sizeof(scanData));
}

bool LIDAR::Connect() {
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "LIDAR: failed to open " << port << ": " << strerror(errno) << "\n";
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "LIDAR: tcgetattr failed: " << strerror(errno) << "\n";
        close(fd);
        fd = -1;
        return false;
    } 
    
    cfsetospeed(&tty, B460800);
    cfsetispeed(&tty, B460800);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_iflag &= ~(IGNBRK | IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 10; // 1 second timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "LIDAR: tcsetattr failed: " << strerror(errno) << "\n";
        close(fd);
        fd = -1;
        return false;
    }

    tcflush(fd, TCIOFLUSH);

    // Reset the device and wait for it to boot
    sendCommand(CMD_RESET);
    usleep(2000000); // 2 seconds
    tcflush(fd, TCIFLUSH);

    std::cout << "LIDAR: device ready\n";
    return true;
}

void LIDAR::Disconnect() {
    if (fd >= 0) {
        uint8_t stop[2] = { CMD_SYNC, CMD_STOP };
        write(fd, stop, 2);
        usleep(20000);
        close(fd);
        fd = -1;
    }
}

bool LIDAR::sendCommand(uint8_t cmd) {
    uint8_t buf[2] = { CMD_SYNC, cmd };
    return write(fd, buf, 2) == 2;
}

bool LIDAR::readDescriptor() {
    // Search for the two sync bytes 0xA5 0x5A rather than assuming alignment
    uint8_t b = 0;
    while (true) {
        if (read(fd, &b, 1) != 1) {
            std::cerr << "LIDAR: timeout waiting for descriptor sync1\n";
            return false;
        }
        if (b != ANS_SYNC1) continue;

        if (read(fd, &b, 1) != 1) {
            std::cerr << "LIDAR: timeout waiting for descriptor sync2\n";
            return false;
        }
        if (b == ANS_SYNC2) break;
    }

    // Read remaining 5 bytes of the descriptor
    uint8_t rest[5];
    int got = 0;
    while (got < 5) {
        int n = static_cast<int>(read(fd, rest + got, static_cast<size_t>(5 - got)));
        if (n <= 0) {
            std::cerr << "LIDAR: failed reading descriptor body\n";
            return false;
        }
        got += n;
    }

    std::cout << "LIDAR: descriptor: A5 5A";
    for (int i = 0; i < 5; i++) printf(" %02X", rest[i]);
    std::cout << "\n";

    return true;
}

// Reads one complete scan revolution (360Â°) from the device.
bool LIDAR::Update() {
    if (fd < 0) return false;
    
    tcflush(fd, TCIFLUSH);
    if (!sendCommand(CMD_SCAN)) return false;
    usleep(100000); // wait for descriptor
    if (!readDescriptor()) return false;

    memset(scanData, 0, sizeof(scanData));

    std::cout << "LIDAR: reading scan packets...\n";
    bool firstRevStart = false;
    int packetCount = 0;

    while (true) {
        uint8_t pkt[5];
        int got = 0;
        while (got < 5) {
            int n = static_cast<int>(read(fd, pkt + got, static_cast<size_t>(5 - got)));
            if (n <= 0) return false;
            got += n;
        }

	bool startFlag         =  pkt[0]        & 0x01;
	bool invertedStartFlag = (pkt[0] >> 1) & 0x01;
        bool checkBit          =  pkt[1]        & 0x01;

        packetCount++;

        // Both flags must be complementary and check bit must be set
        if ((startFlag == invertedStartFlag) || !checkBit) {
            if (packetCount <= 5)
                printf("LIDAR: invalid packet [%d]: %02X %02X %02X %02X %02X\n",
                       packetCount, pkt[0], pkt[1], pkt[2], pkt[3], pkt[4]);
            continue;
        }

        int   quality  =  pkt[0] >> 2;
        float angle    = static_cast<float>((static_cast<uint16_t>(pkt[2]) << 7) | (pkt[1] >> 1)) / 64.0f;
        float distance = static_cast<float>((static_cast<uint16_t>(pkt[4]) << 8) |  pkt[3])       / 4.0f;

        int angleIdx = static_cast<int>(angle) % SCAN_SIZE;

        if (startFlag) {
            std::cout << "LIDAR: revolution start (packets so far: " << packetCount << ")\n";if (!sendCommand(CMD_SCAN)) return false;


            if (firstRevStart) {
                // Second start flag means we completed one full revolution
                updated = true;
                sendCommand(CMD_STOP);
                return true;
            }
            firstRevStart = true;
        }

        if (firstRevStart && quality > 0 && distance > 0.0f) {
            scanData[angleIdx] = static_cast<int>(distance);
        }
    }
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

