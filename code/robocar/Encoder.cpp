#include "Encoder.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <stdio.h>
// ── Constanten ───────────────────────────────────────────────────
#define PULSES_PER_ROT    330
#define WHEEL_CIRC_MM     204.2f
#define UPDATE_US         10000    // 10ms
#define TIMEOUT_US        300000    // 300ms stilstand

// ── Globals ──────────────────────────────────────────────────────
static volatile int pulseCounts[32] = {0};
static bool irqInitialized = false;

// ── ISR ──────────────────────────────────────────────────────────
static void encoderISR(uint gpio, uint32_t events) {
    if (gpio < 32) pulseCounts[gpio]++;
}

// ── Constructor ──────────────────────────────────────────────────
Encoder::Encoder(int GPIOPin, int GPIOPinRes) {
    this->GPIO           = GPIOPin;
    this->GPIOPinRes     = GPIOPinRes;
    this->LinearVelocity = 0.0f;
    this->DistanceMm     = 0.0f;
    this->lastPulseCount = 0;
    this->lastTime       = time_us_64();
    this->lastPulseTime  = this->lastTime;
    this->Updated        = false;
    this->pulsesWithResolution = PULSES_PER_ROT;

    gpio_init(GPIOPinRes);
    gpio_set_dir(GPIOPinRes, GPIO_IN);
    gpio_pull_up(GPIOPinRes);

    if (!irqInitialized) {
        gpio_set_irq_enabled_with_callback(
            GPIOPinRes,
            GPIO_IRQ_EDGE_RISE,
            true,
            &encoderISR
        );
        irqInitialized = true;
    } else {
        gpio_set_irq_enabled(
            GPIOPinRes,
            GPIO_IRQ_EDGE_RISE,
            true
        );
    }
}

// ── Update ───────────────────────────────────────────────────────
bool Encoder::Update() {
    uint64_t now     = time_us_64();
    uint64_t elapsed = now - lastTime;

    if (elapsed < UPDATE_US) return false;
    lastTime = now;

    int current = pulseCounts[GPIOPinRes];
    int delta   = current - lastPulseCount;
    lastPulseCount = current;

    if (delta > 0) lastPulseTime = now;

    if ((now - lastPulseTime) > TIMEOUT_US) {
        LinearVelocity = 0.0f;
        Updated = true;
        return true;
    }

    if (delta == 0) {
        LinearVelocity = 0.0f;
        Updated = true;
        return true;
    }

   
    float rotations  = (float)delta / (pulsesWithResolution);
    float distanceMm = rotations * WHEEL_CIRC_MM;
    float timeSec    = elapsed / 1000000.0f;

    LinearVelocity = distanceMm / timeSec;
    DistanceMm    += distanceMm;
    Updated        = true;

    return true;
}

// ── Reset ────────────────────────────────────────────────────────
void Encoder::Reset() {
    pulseCounts[GPIOPinRes] = 0;
    lastPulseCount          = 0;
    LinearVelocity          = 0.0f;
    DistanceMm              = 0.0f;
    Updated                 = false;
}

// ── Getters ──────────────────────────────────────────────────────
float Encoder::GetLinVelocity() const { return LinearVelocity; }
float Encoder::GetDistanceMm()  const { return DistanceMm;     }
int   Encoder::GetGpio()        const { return GPIO;           }
int   Encoder::GetGpioPinRes()  const { return pulseCounts[GPIOPinRes];    }