#include "PicoUARTHandler.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include "hardware/watchdog.h"

volatile char    PicoUARTHandler::rxBuffer[128] = {0};
volatile int     PicoUARTHandler::rxPos         = 0;
volatile bool    PicoUARTHandler::messageReady  = false;
PicoUARTHandler* PicoUARTHandler::instance      = nullptr;

PicoUARTHandler::PicoUARTHandler(SensorHub& sensorHub)
    : sensorHub(sensorHub)
    , pendingCmd(0.0f, 0.0f)
    , hasPendingCmd(false)
    , lastCmdTimeMs(0)
    , lastPushMs(0)
    , reversalPending(false)
    , reversalCmd(0.0f, 0.0f)
    , reversalStartMs(0)
    , lastLinearSign(0.0f)
{
    instance = this;
}

void PicoUARTHandler::Init()
{
    uart_init(PICO_UART_ID, PICO_UART_BAUD);
    gpio_set_function(PICO_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(PICO_UART_ID, false, false);
    uart_set_format(PICO_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(PICO_UART_ID, false);

    int uart_irq = (PICO_UART_ID == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq, UartRxIrqHandler);
    irq_set_enabled(uart_irq, true);
    uart_set_irq_enables(PICO_UART_ID, true, false);
}

void PicoUARTHandler::UartRxIrqHandler()
{
    while (uart_is_readable(PICO_UART_ID)) {
        char c = uart_getc(PICO_UART_ID);
        if (c == '\r') continue;
        if (c == '\n') {
            rxBuffer[rxPos] = '\0';
            rxPos = 0;
            messageReady = true;
        } else if (rxPos < (int)sizeof(rxBuffer) - 1) {
            rxBuffer[rxPos++] = c;
        } else {
            // Buffer overflow protection — reset on oversized input stream
            rxPos = 0;
        }
    }
}

bool PicoUARTHandler::Tick(DriveCommand& out)
{
    // 1) Process incoming message
    if (messageReady) {
        char lineBuffer[128];
        strncpy(lineBuffer, (const char*)rxBuffer, sizeof(lineBuffer));
        lineBuffer[sizeof(lineBuffer)-1] = '\0';
        messageReady = false;
        handleLine(lineBuffer);
    }

    // 2) Push sensor data every PUSH_INTERVAL_MS
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - lastPushMs >= PUSH_INTERVAL_MS) {
        sensorHub.UpdateSensors();
        pushData();
        lastPushMs = now;
    }

    // 3) Richtingswissel: wacht tot de wielen echt stilstaan voordat we het
    //    omgekeerde commando vrijgeven. Niet-blokkerend — Tick blijft draaien.
    //    Zolang we wachten houden we de robot gestopt. De single-channel
    //    encoders kunnen geen richting meten, dus dit is de enige manier om
    //    te voorkomen dat het commando-teken op de Pi5 tegengesteld raakt aan
    //    de werkelijke (uitrollende) beweging.
    if (reversalPending) {
        float spdL = fabsf(sensorHub.GetSpeedLeft());
        float spdR = fabsf(sensorHub.GetSpeedRight());
        bool  stil    = (spdL < REVERSAL_STILL_MM_S && spdR < REVERSAL_STILL_MM_S);
        bool  timeout = (now - reversalStartMs) > REVERSAL_MAX_MS;

        if (stil || timeout) {
            // Wielen staan stil (of veiligheidstimeout) → geef het bewaarde
            // commando vrij en keer terug naar normale werking.
            pendingCmd      = reversalCmd;
            hasPendingCmd   = false;
            reversalPending = false;
            lastCmdTimeMs   = now;

            float relLin   = reversalCmd.GetLinVelocity();
            lastLinearSign = (relLin >  0.1f) ?  1.0f
                           : (relLin < -0.1f) ? -1.0f : 0.0f;

            out = reversalCmd;
            return true;
        } else {
            // Nog niet stil → blijf de motoren stoppen.
            out = DriveCommand(0.0f, 0.0f);
            return true;
        }
    }

    // 4) Forward pending command
    if (hasPendingCmd) {
        out           = pendingCmd;
        hasPendingCmd = false;
        return true;
    }

    // 5) Command timeout → stop robot
    if (lastCmdTimeMs > 0 && (now - lastCmdTimeMs) > PICO_CMD_TIMEOUT_MS) {
        lastCmdTimeMs = 0;
        out = DriveCommand(0.0f, 0.0f);
        return true;
    }

    return false;
}

void PicoUARTHandler::pushData()
{
    // Use fixed-width formatting for easier Pi5 synchronization
    // Format: DATA:<encL>,<encR>,<yaw>,<omega>\n
    // Use standard decimal formatting (no scientific notation)
    float encL  = sensorHub.GetSpeedLeft();
    float encR  = sensorHub.GetSpeedRight();
    float yaw   = sensorHub.GetCurrentYaw();
    float omega = sensorHub.GetAngVelocity();

    // Clamp NaN/Inf to 0 to prevent invalid UART output strings
    if (encL  != encL  || encL  > 9999.0f || encL  < -9999.0f) encL  = 0.0f;
    if (encR  != encR  || encR  > 9999.0f || encR  < -9999.0f) encR  = 0.0f;
    if (yaw   != yaw   || yaw   >  360.0f || yaw   < -360.0f)  yaw   = 0.0f;
    if (omega != omega || omega >  720.0f || omega < -720.0f) omega = 0.0f;

    char buf[64];
    snprintf(buf, sizeof(buf), "DATA:%.1f,%.1f,%.1f,%.1f\n",
             encL, encR, yaw, omega);
    send(buf);
}

void PicoUARTHandler::handleLine(const char* line)
{
    // STOP command — highest priority, execute immediately.
    // Een expliciete STOP heeft altijd voorrang: een eventuele lopende
    // richtingswissel wordt afgebroken en de richting-state gereset.
    if (strcmp(line, "STOP") == 0) {
        pendingCmd      = DriveCommand(0.0f, 0.0f);
        hasPendingCmd   = true;
        reversalPending = false;
        lastLinearSign  = 0.0f;
        lastCmdTimeMs   = to_ms_since_boot(get_absolute_time());
        send("ACK:STOP\n");
        return;
    }

    if (strcmp(line, "REBOOT") == 0) {
        send("ACK:REBOOT\n");
        sleep_ms(100);
        watchdog_reboot(0, 0, 0);
        return;
    }

    if (strncmp(line, "CMD:", 4) == 0) {
        float lin = 0.0f, ang = 0.0f;
        if (parseCmd(line, lin, ang)) {
            float newSign = (lin >  0.1f) ?  1.0f
                          : (lin < -0.1f) ? -1.0f : 0.0f;

            // Richtingsomkering: nieuw linear-teken is tegengesteld aan het
            // laatst doorgegeven teken. Een nul (stoppen of pure draai) telt
            // NIET als omkering en mag direct door.
            bool isReversal = (lastLinearSign > 0.0f && newSign < 0.0f) ||
                              (lastLinearSign < 0.0f && newSign > 0.0f);

            if (isReversal) {
                // Bewaar het doelcommando en stop eerst tot de wielen stilstaan.
                reversalCmd     = DriveCommand(lin, ang);
                reversalPending = true;
                reversalStartMs = to_ms_since_boot(get_absolute_time());
                pendingCmd      = DriveCommand(0.0f, 0.0f);
                hasPendingCmd   = true;
            } else {
                pendingCmd     = DriveCommand(lin, ang);
                hasPendingCmd  = true;
                lastLinearSign = newSign;
            }
            lastCmdTimeMs = to_ms_since_boot(get_absolute_time());

            char ack[32];
            snprintf(ack, sizeof(ack), "ACK:OK:lin=%.3f,ang=%.3f\n", lin, ang);
            send(ack);
        } else {
            send("ERR:BAD_CMD\n");
        }
        return;
    }

    // Unknown message — silently ignore to avoid UART spam
}


bool PicoUARTHandler::parseCmd(const char* line, float& lin, float& ang)
{
    const char* data = line + 4;
    char* endPtr = nullptr;
    lin = strtof(data, &endPtr);
    if (endPtr == data || *endPtr != ',') return false;
    const char* angStart = endPtr + 1;
    ang = strtof(angStart, &endPtr);
    return (endPtr != angStart);
}

void PicoUARTHandler::send(const char* msg)
{
    uart_puts(PICO_UART_ID, msg);
}