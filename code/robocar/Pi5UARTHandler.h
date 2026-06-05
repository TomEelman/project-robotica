#pragma once

#include <string>

struct SensorData {
    float speedLeft;    // mm/s
    float speedRight;   // mm/s
    float yawDegrees;   // graden
    float angleSpeed;   // graden/s
    bool  valid;        // true = minstens één DATA-pakket ontvangen
};

struct CommandAck {
    bool  valid;
    float lin;
    float ang;
};

class Pi5UARTHandler {
public:
    explicit Pi5UARTHandler(const std::string& port, int baudrate = 115200);
    ~Pi5UARTHandler();

    bool Open();
    void Close();
    bool IsOpen() const;

    bool ReadData();

    SensorData GetSensorData() const { return lastData; }

    void SendCommand(float lin, float ang);
    void SendStop();
    bool RebootPico();

    const std::string& GetPort() const { return port; }
    int  GetFd()   const { return fd; }

private:
    std::string port;
    int         baudrate;
    int         fd;

    SensorData  lastData{};
    char        lineBuffer[256];
    int         linePos;

    bool ConfigurePort();
    bool ParseDataCommand(const char* cmd);
};