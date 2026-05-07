#include "PicoUARTHandler.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include "hardware/watchdog.h"

// ── Statische variabelen ─────────────────────────────────────
volatile char    PicoUARTHandler::rx_buffer[64] = {0};
volatile int     PicoUARTHandler::rx_pos        = 0;
volatile bool    PicoUARTHandler::bericht_klaar = false;
PicoUARTHandler* PicoUARTHandler::instance      = nullptr;

// ── Constructor ──────────────────────────────────────────────
PicoUARTHandler::PicoUARTHandler(SensorHub& sensorHub)
    : sensorHub(sensorHub)
    , pendingCmd(0.0f, 0.0f)
    , hasPendingCmd(false)
    , lastCmdTimeMs(0)
{
    instance = this;
}

// ── Init: UART0 + IRQ ────────────────────────────────────────
void PicoUARTHandler::Init()
{
    uart_init(PICO_UART_ID, PICO_UART_BAUD);
    gpio_set_function(PICO_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(PICO_UART_ID, false, false);
    uart_set_format(PICO_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(PICO_UART_ID, false);  // byte-voor-byte IRQ

    int uart_irq = (PICO_UART_ID == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq, UartRxIrqHandler);
    irq_set_enabled(uart_irq, true);
    uart_set_irq_enables(PICO_UART_ID, true, false);
}

// ── IRQ handler ──────────────────────────────────────────────
// Identiek aan de oude SensorHub::UartRxIrqHandler —
// verzamelt bytes tot '\n', zet dan bericht_klaar.
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

// ── ConsumePendingCmd ────────────────────────────────────────
// Aanroepen elke loop-iteratie.
// Verwerkt een klaarstaand bericht en geeft CMD door als nodig.
bool PicoUARTHandler::ConsumePendingCmd(DriveCommand& out)
{
    // 1) Verwerk bericht als de IRQ er één klaar heeft gezet
    if (bericht_klaar) {
        char regel[64];
        strncpy(regel, (const char*)rx_buffer, sizeof(regel));
        bericht_klaar = false;
        HandleLine(regel);
    }

    // 2) Geef pending CMD door
    if (hasPendingCmd) {
        out           = pendingCmd;
        hasPendingCmd = false;
        return true;
    }

    // 3) Timeout: geen CMD ontvangen → stuur stop
    if (lastCmdTimeMs > 0) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - lastCmdTimeMs) > PICO_CMD_TIMEOUT_MS) {
            lastCmdTimeMs = 0;
            out = DriveCommand(0.0f, 0.0f);
            return true;
        }
    }

    return false;
}

// ── HandleLine ───────────────────────────────────────────────
void PicoUARTHandler::HandleLine(const char* line)
{
    // reboot commando
    if (strcmp(line, "REBOOT") == 0) {
        Send("ACK:REBOOT\n");
        sleep_ms(100);  // even wachten zodat ACK verstuurd wordt
        watchdog_reboot(0, 0, 0);
        return;
    }

    if (strncmp(line, "GET:", 4) == 0) {
        const char* sensor = line + 4;
        if      (strcmp(sensor, "ENCODER") == 0) SendEncoder();
        else if (strcmp(sensor, "IMU")     == 0) SendIMU();
        else                                     Send("ERR:UNKNOWN_SENSOR\n");
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

// ── SendEncoder ──────────────────────────────────────────────
void PicoUARTHandler::SendEncoder()
{
    char buf[96];
    snprintf(buf, sizeof(buf), "ENCODER:%.2f,%.2f,%.2f,%.2f\n",
        sensorHub.GetSpeedLeft(),
        sensorHub.GetDistanceLeft(),
        sensorHub.GetSpeedRight(),
        sensorHub.GetDistanceRight());
    Send(buf);
}

// ── SendIMU ──────────────────────────────────────────────────
void PicoUARTHandler::SendIMU()
{
    char buf[64];
    snprintf(buf, sizeof(buf), "IMU:%.4f,%.4f\n",
        sensorHub.GetCurrentYaw(),
        sensorHub.GetAngVelocity());
    Send(buf);
}

// ── ParseCmd ─────────────────────────────────────────────────
bool PicoUARTHandler::ParseCmd(const char* line, float& lin, float& ang)
{
    const char* data = line + 4;   // sla "CMD:" over
    char* endPtr = nullptr;

    lin = strtof(data, &endPtr);
    if (endPtr == data || *endPtr != ',') return false;

    const char* angStart = endPtr + 1;
    ang = strtof(angStart, &endPtr);
    if (endPtr == angStart) return false;

    return true;
}

// ── Send ─────────────────────────────────────────────────────
void PicoUARTHandler::Send(const char* msg)
{
    uart_puts(PICO_UART_ID, msg);
}