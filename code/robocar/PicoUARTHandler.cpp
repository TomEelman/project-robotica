#include "PicoUARTHandler.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

// UART0 is GP0/GP1 – dezelfde UART die al gebruikt wordt voor communicatie
#define UART_ID   uart0
#define UART_BAUD 115200

UARTCommandHandler::UARTCommandHandler(Robot& robot, SensorHub& sensorHub)
    : robot(robot)
    , sensorHub(sensorHub)
    , rxLen(0)
    , lastCmd(0.0f, 0.0f)
    , hasCmd(false)
    , lastCmdTimeMs(0)
{
    memset(rxBuf, 0, sizeof(rxBuf));
}

// ─────────────────────────────────────────────
//  Poll  –  aanroepen elke ~10 ms
// ─────────────────────────────────────────────
void UARTCommandHandler::Poll()
{
    // 1) Lees alle beschikbare bytes uit de UART FIFO
    while (uart_is_readable(UART_ID))
    {
        char c = uart_getc(UART_ID);

        if (c == '\n' || c == '\r')
        {
            if (rxLen > 0)
            {
                rxBuf[rxLen] = '\0';
                HandleLine(rxBuf);
                rxLen = 0;
            }
        }
        else
        {
            if (rxLen < (int)sizeof(rxBuf) - 1)
                rxBuf[rxLen++] = c;
            else
                rxLen = 0;   // overflow → reset
        }
    }

    // 2) Timeout-bewaking: geen CMD ontvangen → stop robot
    if (hasCmd)
    {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - lastCmdTimeMs) > CMD_TIMEOUT_MS)
        {
            hasCmd  = false;
            lastCmd = DriveCommand(0.0f, 0.0f);
            // Stuur een expliciete stop zodat de robot niet blijft rijden
            robot.Execute(DriveCommand(0.0f, 0.0f));
        }
    }
}

// ─────────────────────────────────────────────
//  HandleLine  –  verwerk één volledige regel
// ─────────────────────────────────────────────
void UARTCommandHandler::HandleLine(const char* line)
{
    // ── GET-verzoeken (sensordata terugsturen) ──────────────
    if (strncmp(line, "GET:", 4) == 0)
    {
        const char* sensor = line + 4;

        if (strcmp(sensor, "ENCODER") == 0)
            SendEncoder();
        else if (strcmp(sensor, "IMU") == 0)
            SendIMU();
        else
        {
            // Onbekende sensor → foutmelding
            uart_puts(UART_ID, "ERR:UNKNOWN_SENSOR\n");
        }
        return;
    }

    // ── CMD-berichten (DriveCommand ontvangen) ──────────────
    if (strncmp(line, "CMD:", 4) == 0)
    {
        float lin = 0.0f, ang = 0.0f;
        if (ParseCmd(line, lin, ang))
        {
            lastCmd        = DriveCommand(lin, ang);
            hasCmd         = true;
            lastCmdTimeMs  = to_ms_since_boot(get_absolute_time());

            // Voer het commando direct uit
            robot.Execute(lastCmd);

            // Stuur een bevestiging terug (optioneel, handig voor debugging)
            char ack[64];
            snprintf(ack, sizeof(ack), "ACK:%.2f,%.2f\n", lin, ang);
            uart_puts(UART_ID, ack);
        }
        else
        {
            uart_puts(UART_ID, "ERR:BAD_CMD\n");
        }
        return;
    }

    // Onbekend bericht
    uart_puts(UART_ID, "ERR:UNKNOWN\n");
}

// ─────────────────────────────────────────────
//  SendEncoder
// ─────────────────────────────────────────────
void UARTCommandHandler::SendEncoder()
{
    // SensorHub levert snelheid (mm/s) en afstand (mm) per wiel
    float sL = sensorHub.GetSpeedLeft();
    float dL = sensorHub.GetDistanceLeft();
    float sR = sensorHub.GetSpeedRight();
    float dR = sensorHub.GetDistanceRight();

    char buf[96];
    snprintf(buf, sizeof(buf), "ENCODER:%.3f,%.3f,%.3f,%.3f\n", sL, dL, sR, dR);
    uart_puts(UART_ID, buf);
}

// ─────────────────────────────────────────────
//  SendIMU
// ─────────────────────────────────────────────
void UARTCommandHandler::SendIMU()
{
    float yawDeg = sensorHub.GetCurrentYaw();
    float omega  = sensorHub.GetAngVelocity();   // graden/s

    char buf[64];
    snprintf(buf, sizeof(buf), "IMU:%.3f,%.3f\n", yawDeg, omega);
    uart_puts(UART_ID, buf);
}

// ─────────────────────────────────────────────
//  ParseCmd  –  "CMD:lin,ang"
// ─────────────────────────────────────────────
bool UARTCommandHandler::ParseCmd(const char* line, float& lin, float& ang)
{
    // Verwacht formaat: "CMD:lin,ang"
    const char* data = line + 4;   // sla "CMD:" over

    char* endPtr = nullptr;
    lin = strtof(data, &endPtr);

    if (endPtr == data || *endPtr != ',')
        return false;

    const char* angStart = endPtr + 1;
    ang = strtof(angStart, &endPtr);

    if (endPtr == angStart)
        return false;

    return true;
}

// ─────────────────────────────────────────────
//  Getters
// ─────────────────────────────────────────────
bool UARTCommandHandler::HasActiveCommand() const
{
    return hasCmd;
}

DriveCommand UARTCommandHandler::GetLastCommand() const
{
    return lastCmd;
}