#include "PicoUARTHandler.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include "hardware/watchdog.h"

volatile char    PicoUARTHandler::rx_buffer[64] = {0};
volatile int     PicoUARTHandler::rx_pos        = 0;
volatile bool    PicoUARTHandler::bericht_klaar = false;
PicoUARTHandler* PicoUARTHandler::instance      = nullptr;

PicoUARTHandler::PicoUARTHandler(SensorHub& sensorHub)
    : sensorHub(sensorHub)
    , pendingCmd(0.0f, 0.0f)
    , hasPendingCmd(false)
    , lastCmdTimeMs(0)
    , lastPushMs(0)
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

// ── IRQ: verzamel bytes tot '\n' ─────────────────────────────
void PicoUARTHandler::UartRxIrqHandler()
{
    while (uart_is_readable(PICO_UART_ID)) {
        char c = uart_getc(PICO_UART_ID);
        if (c == '\r') continue;
        if (c == '\n') {
            rx_buffer[rx_pos] = '\0';
            rx_pos = 0;
            bericht_klaar = true;
        } else if (rx_pos < (int)sizeof(rx_buffer) - 1) {
            rx_buffer[rx_pos++] = c;
        }
    }
}

// ── Tick: aanroepen elke 10 ms ───────────────────────────────
bool PicoUARTHandler::Tick(DriveCommand& out)
{
    // 1) Verwerk inkomend bericht
    if (bericht_klaar) {
        char regel[64];
        strncpy(regel, (const char*)rx_buffer, sizeof(regel));
        bericht_klaar = false;
        HandleLine(regel);
    }

    // 2) Push sensor-data elke PUSH_INTERVAL_MS
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - lastPushMs >= PUSH_INTERVAL_MS) {
        sensorHub.UpdateSensors();
        PushData();
        lastPushMs = now;
    }

    // 3) Geef pending CMD door
    if (hasPendingCmd) {
        out           = pendingCmd;
        hasPendingCmd = false;
        return true;
    }

    // 4) CMD timeout → stop
    if (lastCmdTimeMs > 0 && (now - lastCmdTimeMs) > PICO_CMD_TIMEOUT_MS) {
        lastCmdTimeMs = 0;
        out = DriveCommand(0.0f, 0.0f);
        return true;
    }

    return false;
}

// ── PushData: stuur "DATA:encL,encR,yaw,omega\n" ─────────────
void PicoUARTHandler::PushData()
{
    char buf[96];
    snprintf(buf, sizeof(buf), "DATA:%.2f,%.2f,%.3f,%.3f\n",
        sensorHub.GetSpeedLeft(),
        sensorHub.GetSpeedRight(),
        sensorHub.GetCurrentYaw(),
        sensorHub.GetAngVelocity());
    Send(buf);
}

// ── HandleLine: verwerk inkomend bericht ─────────────────────
void PicoUARTHandler::HandleLine(const char* line)
{
    if (strcmp(line, "REBOOT") == 0) {
        Send("ACK:REBOOT\n");
        sleep_ms(100);
        watchdog_reboot(0, 0, 0);
        return;
    }

    if (strncmp(line, "CMD:", 4) == 0) {
        float lin = 0.0f, ang = 0.0f;
        if (ParseCmd(line, lin, ang)) {
            pendingCmd    = DriveCommand(lin, ang);
            hasPendingCmd = true;
            lastCmdTimeMs = to_ms_since_boot(get_absolute_time());
            char ack[64];
            snprintf(ack, sizeof(ack), "ACK:%.2f,%.2f\n", lin, ang);
            Send(ack);
        } else {
            Send("ERR:BAD_CMD\n");
        }
        return;
    }

    Send("ERR:UNKNOWN\n");
}

bool PicoUARTHandler::ParseCmd(const char* line, float& lin, float& ang)
{
    const char* data = line + 4;
    char* endPtr = nullptr;
    lin = strtof(data, &endPtr);
    if (endPtr == data || *endPtr != ',') return false;
    const char* angStart = endPtr + 1;
    ang = strtof(angStart, &endPtr);
    return (endPtr != angStart);
}

void PicoUARTHandler::Send(const char* msg)
{
    uart_puts(PICO_UART_ID, msg);
}