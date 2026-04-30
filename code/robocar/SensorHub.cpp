#include "SensorHub.h"
#include <string.h>
#include <stdio.h>

// ── Statische variabelen initialiseren ──────────────────────
volatile char  SensorHub::rx_buffer[64] = {0};
volatile int   SensorHub::rx_pos        = 0;
volatile bool  SensorHub::bericht_klaar = false;
SensorHub*     SensorHub::instance      = nullptr;

// ── Constructor ─────────────────────────────────────────────
SensorHub::SensorHub(int encLeft, int encLeftRes,
                     int encRight, int encRightRes,
                     int imuSDAPin, int imuSCLPin)
    : encoderLeft(encLeft, encLeftRes),
      encoderRight(encRight, encRightRes),
      imu(imuSDAPin, imuSCLPin, 0x28),
      sensorsUpdated(false)
{
    instance = this;  // Sla de instantie op voor de IRQ handler
}

// ── UART initialisatie ───────────────────────────────────────
void SensorHub::InitUart() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, false);  // Elke byte triggert direct een IRQ

    int uart_irq = (UART_ID == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(uart_irq, UartRxIrqHandler);
    irq_set_enabled(uart_irq, true);
    uart_set_irq_enables(UART_ID, true, false);  // RX=true, TX=false
}

// ── Statische IRQ handler ────────────────────────────────────
void SensorHub::UartRxIrqHandler() {
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);

        if (c == '\r') continue;  // ← Negeer \r

        if (c == '\n') {
            rx_buffer[rx_pos] = '\0';
            rx_pos = 0;
            bericht_klaar = true;
        } else if (rx_pos < (int)sizeof(rx_buffer) - 1) {
            rx_buffer[rx_pos++] = c;
        }
    }
}

// ── Wordt aangeroepen in de main loop ───────────────────────
void SensorHub::HandleUart() {
    if (!bericht_klaar) return;

    // Kopieer buffer voordat nieuwe IRQ hem overschrijft
    char verzoek[64];
    strncpy(verzoek, (const char*)rx_buffer, sizeof(verzoek));
    bericht_klaar = false;               // Haal de flag weg

    VerwerkVerzoek(verzoek);
}

// ── Verzoek verwerken en antwoord sturen ────────────────────
void SensorHub::VerwerkVerzoek(const char* verzoek) {
    UpdateSensors();
    char antwoord[128];

    if (strcmp(verzoek, "GET:ENCODER") == 0) {
        snprintf(antwoord, sizeof(antwoord), "ENCODER:%.2f,%.2f,%.2f,%.2f\n",
            GetSpeedLeft(),
            GetDistanceLeft(),
            GetSpeedRight(),
            GetDistanceRight());

    } else if (strcmp(verzoek, "GET:IMU") == 0) {
        snprintf(antwoord, sizeof(antwoord), "IMU:%.4f,%.4f\n",
            GetCurrentYaw(),
            GetAngVelocity());

    } else {
        snprintf(antwoord, sizeof(antwoord), "ERROR:ONBEKEND\n");
    }

    StuurAntwoord(antwoord);
}

// ── Antwoord via UART versturen ──────────────────────────────
void SensorHub::StuurAntwoord(const char* antwoord) {
    uart_puts(UART_ID, antwoord);
}

// ── Sensor methodes ─────────────────────────────────────────
bool SensorHub::UpdateSensors() {
    bool imuOk   = imu.Update();
    bool leftOk  = encoderLeft.Update();
    bool rightOk = encoderRight.Update();
    sensorsUpdated = imuOk && leftOk && rightOk;
    return sensorsUpdated;
}

float SensorHub::GetSpeedLeft()     const { return encoderLeft.GetLinVelocity(); }
float SensorHub::GetDistanceLeft()  const { return encoderLeft.GetDistanceMm(); }
float SensorHub::GetSpeedRight()    const { return encoderRight.GetLinVelocity(); }
float SensorHub::GetDistanceRight() const { return encoderRight.GetDistanceMm(); }
float SensorHub::GetCurrentYaw()    const { return imu.GetCurrentYaw(); }
float SensorHub::GetAngVelocity()   const { return imu.GetAngVelocity(); }
DateTime SensorHub::GetLastUpdate() const { return lastUpdate; }