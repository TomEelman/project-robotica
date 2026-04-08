#include "Encoder.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include <cstdio>

// ── Encoder constanten ───────────────────────────────────────────
#define RESOLUTION          330
#define WHEEL_DIAMETER_MM   65.0f
#define WHEEL_CIRCUMFERENCE (3.14159f * WHEEL_DIAMETER_MM)

// ── Globals ──────────────────────────────────────────────────────
static bool irqInitialized = false;

// mapping: GPIO → Encoder instance
static Encoder* encoderInstances[32] = {nullptr};

// pulse counters per GPIO (interrupt safe genoeg op Pico)
static volatile int pulseCounts[32] = {0};

// ── ISR ──────────────────────────────────────────────────────────
static void encoderISR(uint gpio, uint32_t events) {
    if (gpio < 32) {
        Encoder* enc = encoderInstances[gpio];
        if (enc != nullptr) {
            pulseCounts[gpio]++;
        }
    }
}

// ── Constructor ──────────────────────────────────────────────────
Encoder::Encoder(int GPIOPin, int GPIOPinRes) {
    this->GPIO = GPIOPin;
    this->GPIOPinRes = GPIOPinRes;

    // Koppel deze instance aan GPIO
    encoderInstances[GPIOPinRes] = this;

    gpio_init(GPIOPinRes);
    gpio_set_dir(GPIOPinRes, GPIO_IN);
    gpio_pull_up(GPIOPinRes);

    // Slechts 1x globale callback registreren
    if (!irqInitialized) {
        gpio_set_irq_enabled_with_callback(
            GPIOPinRes,
            GPIO_IRQ_EDGE_RISE,
            true,
            &encoderISR
        );
        irqInitialized = true;
    }

    // Voor deze pin interrupt aanzetten
    gpio_set_irq_enabled(GPIOPinRes, GPIO_IRQ_EDGE_RISE, true);

    // Init state
    lastTime = time_us_64();
    lastPulseCount = 0;
    lastPulseTime = lastTime;
    LinearVelocity = 0.0f;

    pulsesWithResolution = RESOLUTION;
}

// ── Update ───────────────────────────────────────────────────────
bool Encoder::Update() {
    uint64_t now = time_us_64();
    uint64_t elapsed = now - lastTime;

    // update elke 100 ms
    if (elapsed < 100000) {
        return false;
    }

    lastTime = now;

    int currentPulses = pulseCounts[GPIOPinRes];
    int delta = currentPulses - lastPulseCount;
    lastPulseCount = currentPulses;

    // beweging gedetecteerd
    if (delta > 0) {
        lastPulseTime = now;
    }

    // timeout → stilstand
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

    LinearVelocity = distance_mm / time_s;

    return true;
}

// ── Reset ────────────────────────────────────────────────────────
void Encoder::Reset() {
    pulseCounts[GPIOPinRes] = 0;
    lastPulseCount = 0;
    LinearVelocity = 0.0f;
}

// ── Getters ──────────────────────────────────────────────────────
int Encoder::GetPulses() const {
    return pulseCounts[GPIOPinRes];
}

int Encoder::GetPulsesWR() const {
    return pulsesWithResolution;
}

float Encoder::GetLinVelocity() const {
    return LinearVelocity;
}

int Encoder::GetGpio() const {
    return GPIO;
}

int Encoder::GetGpioPinRes() const {
    return GPIOPinRes;
}