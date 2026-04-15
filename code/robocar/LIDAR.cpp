#include "LIDAR.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <iostream>
#include <cstdio>

static constexpr uint8_t CMD_SYNC  = 0xA5;  // startbyte voor elk commando
static constexpr uint8_t CMD_SCAN  = 0x20;  // "begin scannen"
static constexpr uint8_t CMD_STOP  = 0x25;  // "stop scannen"
static constexpr uint8_t CMD_RESET = 0x40;  // "herstart apparaat"
static constexpr uint8_t ANS_SYNC1 = 0xA5;  // eerste antwoord-syncbyte
static constexpr uint8_t ANS_SYNC2 = 0x5A;  // tweede antwoord-syncbyte

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
    memset(scanData, 0, sizeof(scanData)); // initialiseert de scandata array met nullen
}

// Neemt een referentie zodat de wijzigingen ook buiten de functie gelden
void configure_tty(struct termios& tty) {
    // Stelt de baudrate in op 460800, dit is RPLIDAR C1 specifiek
    cfsetospeed(&tty, B460800);
    cfsetispeed(&tty, B460800);

    // Stel 8 databits in: wis eerst de huidige bitgrootte (CSIZE), zet dan CS8 (8 bits)
    tty.c_cflag = (tty.c_cflag & ~static_cast<tcflag_t>(CSIZE)) | CS8;

    // CLOCAL: negeer modemstatussignalen (geen CD/DTR/RTS nodig)
    // CREAD:  schakel de ontvanger in zodat data gelezen kan worden
    tty.c_cflag |= (CLOCAL | CREAD);

    // PARENB:  geen pariteitsbit genereren/controleren
    // CSTOPB:  gebruik 1 stopbit in plaats van 2
    // CRTSCTS: geen hardware flow control (RTS/CTS pinnen niet gebruiken)
    tty.c_cflag &= ~static_cast<tcflag_t>(PARENB | CSTOPB | CRTSCTS);

    // IGNBRK: verwerk break-signalen normaal (niet negeren)
    // IXON:   geen software flow control bij verzenden (geen XON/XOFF)
    // IXOFF:  geen software flow control bij ontvangen
    // IXANY:  elk teken mag de output niet hervatten
    tty.c_iflag &= ~static_cast<tcflag_t>(IGNBRK | IXON | IXOFF | IXANY);

    // Zet lokale modus & output verwerking op 0 (raw mode)
    tty.c_lflag = 0;
    tty.c_oflag = 0;

    // Blokkeer read() totdat minstens 1 byte beschikbaar is
    tty.c_cc[VMIN]  = 1;

    // Timeout van 1 seconde (waarde in tienden van seconden: 10 = 1.0s)
    // Als na 1 seconde nog geen data is, keert read() terug met 0 bytes
    tty.c_cc[VTIME] = 10;
}

bool LIDAR::Connect() {
    /*
    Flags:
        O_RDWR   zet de port naar "ReaD WRite"
        O_NOCTTY zorgt dat het geen controlling terminal wordt
        O_SYNC   zorgt dat writes wachten tot het apparaat ze heeft ontvangen
    */
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0) {
        std::cerr << "LIDAR: failed to open " << port << ": " << strerror(errno) << "\n";
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    // Leest de huidige terminalinstellingen in tty
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "LIDAR: tcgetattr failed: " << strerror(errno) << "\n";
        close(fd);
        fd = -1;
        return false;
    }

    configure_tty(tty);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "LIDAR: tcsetattr failed: " << strerror(errno) << "\n";
        close(fd);
        fd = -1;
        return false;
    }

    tcflush(fd, TCIOFLUSH);

    sendCommand(CMD_RESET);
    sleep(2);
    tcflush(fd, TCIFLUSH);

    std::cout << "LIDAR: device configured and ready\n";
    return true;
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

// Reads one complete scan revolution (360°) from the device.
bool LIDAR::Update() {
    if (fd < 0) return false;
    
    tcflush(fd, TCIFLUSH);
    if (!sendCommand(CMD_SCAN)) return false;

    sleep(1);
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

void LIDAR::Disconnect() {
    if (fd >= 0) {
        // Stuur stopcommando zodat het apparaat netjes stopt met draaien
        uint8_t stop[2] = { CMD_SYNC, CMD_STOP };
        write(fd, stop, 2);
        usleep(20000); // 20ms wachten zodat het apparaat stopt
        close(fd);
        fd = -1;
    }
}
