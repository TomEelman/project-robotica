#include "Encoder.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <stdio.h>

// ── Constanten ───────────────────────────────────────────────────
#define PULSES_PER_ROT   330
#define WHEEL_CIRC_MM    204.2f
#define UPDATE_US        50000     // 50 ms → 20 Hz update
#define TIMEOUT_US       300000    // 300 ms zonder pulsen → v = 0

// ── Globals ──────────────────────────────────────────────────────
static volatile int  pulseCounts[32] = {0};
static bool          irqInitialized  = false;

// ── ISR ──────────────────────────────────────────────────────────
static void encoderISR(uint gpio, uint32_t events) {
    if (gpio < 32) pulseCounts[gpio]++;
}

// ── Constructor ──────────────────────────────────────────────────
Encoder::Encoder(int GPIOPin, int GPIOPinResolution) {
    GPIO                 = GPIOPin;
    GPIOPinRes           = GPIOPinResolution;
    pulsesWithResolution = PULSES_PER_ROT;

    LinearVelocity = 0.0f;
    DistanceMm     = 0.0f;
    lastPulseCount = 0;
    lastTime       = time_us_64();
    lastPulseTime  = lastTime;
    Updated        = false;
    freshData      = false;

    gpio_init(GPIOPinRes);
    gpio_set_dir(GPIOPinRes, GPIO_IN);
    gpio_pull_up(GPIOPinRes);

    if (!irqInitialized) {
        gpio_set_irq_enabled_with_callback(
            GPIOPinRes, GPIO_IRQ_EDGE_RISE, true, &encoderISR);
        irqInitialized = true;
    } else {
        gpio_set_irq_enabled(GPIOPinRes, GPIO_IRQ_EDGE_RISE, true);
    }
}

// ── Update ───────────────────────────────────────────────────────
// Returnt true als er een nieuw sample is geproduceerd.
bool Encoder::Update() {
    uint64_t now     = time_us_64();
    uint64_t elapsed = now - lastTime;

    if (elapsed < UPDATE_US) {
        return false;
    }
    lastTime = now;

    int current = pulseCounts[GPIOPinRes];
    int delta   = current - lastPulseCount;
    lastPulseCount = current;

    if (delta > 0) {
        lastPulseTime = now;
    }

    // Te lang geen pulsen → wiel staat stil
    if ((now - lastPulseTime) > TIMEOUT_US) {
        LinearVelocity = 0.0f;
        Updated        = true;
        freshData      = true;
        return true;
    }

    // Snelheid berekenen (delta == 0 levert netjes 0 mm/s op)
    float rotations  = (float)delta / (float)pulsesWithResolution;
    float distanceMm = rotations * WHEEL_CIRC_MM;
    float timeSec    = elapsed / 1000000.0f;

    LinearVelocity = distanceMm / timeSec;
    DistanceMm    += distanceMm;
    Updated        = true;
    freshData      = true;
    return true;
}

// ── Reset ────────────────────────────────────────────────────────
void Encoder::Reset() {
    pulseCounts[GPIOPinRes] = 0;
    lastPulseCount          = 0;
    LinearVelocity          = 0.0f;
    DistanceMm              = 0.0f;
    Updated                 = false;
    freshData               = false;
    lastTime                = time_us_64();
    lastPulseTime           = lastTime;
}

// ── Getters ──────────────────────────────────────────────────────
float Encoder::GetLinVelocity() const { return LinearVelocity; }
float Encoder::GetDistanceMm()  const { return DistanceMm;     }
int   Encoder::GetGpio()        const { return GPIO;           }
int   Encoder::GetGpioPinRes()  const { return GPIOPinRes;     }

// HasFreshData() / ConsumeFreshFlag() zijn inline in de header.