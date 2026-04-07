#include "Encoder.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include <cstdio>
#include "hardware/gpio.h"

// ── Encoder constanten ───────────────────────────────────────────
#define RESOLUTION          330
#define WHEEL_DIAMETER_MM   65.0f
#define WHEEL_CIRCUMFERENCE (3.14159f * WHEEL_DIAMETER_MM)

// ── Statische pulse tellers (interrupt safe) ─────────────────────
static volatile int pulseCounts[32] = {0};

// ── Interrupt handler ────────────────────────────────────────────
static void encoderISR(uint gpio, uint32_t events) {
    if (gpio < 32) {
        pulseCounts[gpio]++;
    }
}

// ── Constructor ──────────────────────────────────────────────────
Encoder::Encoder(int GPIOPin, int GPIOPinRes) {
    this->GPIO = GPIOPin;
    this->GPIOPinRes = GPIOPinRes;

    pulses = 0;
    pulsesWithResolution = RESOLUTION;
    LinearVelocity = 0.0f;
    Updaded = false;

    // High-resolution pin (groene draad)
    gpio_init(this->GPIOPinRes);
    gpio_set_dir(this->GPIOPinRes, GPIO_IN);
    gpio_pull_up(this->GPIOPinRes);

    gpio_set_irq_enabled_with_callback(
        GPIOPinRes,
        GPIO_IRQ_EDGE_RISE,
        true,
        &encoderISR
    );
}

// ── Update ───────────────────────────────────────────────────────
bool Encoder::Update() {
    static uint64_t lastTime = 0;
    static int lastPulseCount = 0;
    static uint64_t lastPulseTime = 0;

    uint64_t now = time_us_64();
    uint64_t elapsed = now - lastTime;

    if (elapsed < 100000) return false;

    lastTime = now;

    int currentPulses = pulseCounts[GPIOPinRes];
    int delta = currentPulses - lastPulseCount;
    lastPulseCount = currentPulses;

    if (delta > 0) {
        lastPulseTime = now;
    }

    // TIMEOUT → motor staat stil
    if ((now - lastPulseTime) > 200000) {
        LinearVelocity = 0.0f;
        return true;
    }

    if (delta == 0) {
        LinearVelocity = 0.0f;
        return true;
    }

    float time_s = elapsed / 1000000.0f;

    float rotations = (float)delta / pulsesWithResolution;
    float distance_mm = rotations * WHEEL_CIRCUMFERENCE;

    float velocity = distance_mm / time_s;

    LinearVelocity = velocity;

    return true;
}

void Encoder::Reset() {
    pulses = 0;
    pulseCounts[GPIOPinRes] = 0;
    LinearVelocity = 0.0f;
    Updaded = false;
}

int Encoder::GetPulses()const {
    return pulses;
}

int Encoder::GetPulsesWR()const {
    return pulsesWithResolution;
}

float Encoder::GetLinVelocity()const {
    return LinearVelocity;
}

int Encoder::GetGpio() const{
    return GPIO;
}

int Encoder::GetGpioPinRes()const {
    return GPIOPinRes;
}