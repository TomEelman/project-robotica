#include "Encoder.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// Encoder hardware constants
static constexpr int   PULSES_PER_ROTATION  = 330; // from datasheet
static constexpr float WHEEL_CIRCUMFERENCE_MM = 204.2f; // 65 mm diameter


static constexpr uint64_t UPDATE_INTERVAL_US = 50'000; // 50ms update rate

// if no pulse arrives within this window the wheel is considered stopped.
static constexpr uint64_t PULSE_TIMEOUT_US = 300'000;

static volatile int pulseCounts[32] = {0};
static bool         irqInitialized  = false;



static void EncoderISR(uint gpio, uint32_t /*events*/)
{
    if (gpio < 32) pulseCounts[gpio]++;
}

Encoder::Encoder(int gpioPinRes)
    : 
      gpioPinRes(gpioPinRes),
      pulsesPerRotation(PULSES_PER_ROTATION),
      linearVelocity(0.0f),
      distanceMm(0.0f),
      lastPulseCount(0),
      lastTime(time_us_64()),
      lastPulseTime(time_us_64()),
      updated(false),
      freshData(false)
{
    gpio_init(gpioPinRes);
    gpio_set_dir(gpioPinRes, GPIO_IN);
    gpio_pull_up(gpioPinRes);

    if (!irqInitialized) {
        gpio_set_irq_enabled_with_callback(
            gpioPinRes, GPIO_IRQ_EDGE_RISE, true, &EncoderISR);
        irqInitialized = true;
    } else {
        gpio_set_irq_enabled(gpioPinRes, GPIO_IRQ_EDGE_RISE, true);
    }
}

bool Encoder::Update()
{
    uint64_t now     = time_us_64();
    uint64_t elapsed = now - lastTime;

    if (elapsed < UPDATE_INTERVAL_US) {
        return false;
    }

    lastTime = now;

    int current = pulseCounts[gpioPinRes]; 
    int delta   = current - lastPulseCount;
    lastPulseCount = current;

    if (delta > 0) {
        lastPulseTime = now;
    }

    if ((now - lastPulseTime) > PULSE_TIMEOUT_US) {
        linearVelocity = 0.0f;
        updated        = true;
        freshData      = true;
        return true;
    }

    // Convert pulse count to mm/s.
    // rotations = delta / ppr;  distance = rotations * circumference;  veolcity = distance / dt
    float rotations  = static_cast<float>(delta) / static_cast<float>(pulsesPerRotation);
    float distanceDelta = rotations * WHEEL_CIRCUMFERENCE_MM;
    float dtSeconds  = static_cast<float>(elapsed) / 1'000'000.0f;

    linearVelocity  = distanceDelta / dtSeconds;
    distanceMm     += distanceDelta;
    updated         = true;
    freshData       = true;
    return true;
}

void Encoder::Reset()
{
    pulseCounts[gpioPinRes] = 0;
    lastPulseCount          = 0;
    linearVelocity          = 0.0f;
    distanceMm              = 0.0f;
    updated                 = false;
    freshData               = false;
    lastTime                = time_us_64();
    lastPulseTime           = lastTime;
}

float Encoder::GetLinVelocity() const { return linearVelocity; }
float Encoder::GetDistanceMm()  const { return distanceMm;     }
// int   Encoder::GetGpio()        const { return gpio;           }
// int   Encoder::GetGpioPinRes()  const { return gpioPinRes;     }