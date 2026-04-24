#include "Motor.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// ── Hulpfunctie (intern) ─────────────────────────────────────────
static void initPwmPin(int pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 1.907f);  // ~1 kHz bij wrap=65535
    pwm_config_set_wrap(&cfg, 65535);
    pwm_init(slice, &cfg, true);
}

static void setDuty(int pin, float procent) {
    // procent: -100.0 t/m 100.0, hier altijd positief meegeven
    if (procent < -255.0f) procent = -255.0f;
    if (procent > 255.0f) procent = 255.0f;
    unsigned short level = (unsigned short)(65535.0f * procent / 255.0f);
    pwm_set_gpio_level(pin, level);
}

// ── Constructor ──────────────────────────────────────────────────
Motor::Motor(int PWMPin, int ForwardPin, int BackwardPin) {
    pwmPin      = PWMPin;
    forwardPin  = ForwardPin;
    backwardPin = BackwardPin;
    speed       = 0.0f;

    // Richting-pinnen als output, standaard laag
    gpio_init(forwardPin);
    gpio_set_dir(forwardPin, GPIO_OUT);
    gpio_put(forwardPin, 0);

    gpio_init(backwardPin);
    gpio_set_dir(backwardPin, GPIO_OUT);
    gpio_put(backwardPin, 0);

    // PWM initialiseren
    initPwmPin(pwmPin);
    setDuty(pwmPin, 0.0f);
}

// ── Getters / Setters ────────────────────────────────────────────
float Motor::GetSpeed() {
    return speed;
}

void Motor::SetSpeed(float Speed) {
    speed = Speed;

    if (Speed > 0.0f) {
        // Vooruit
        gpio_put(forwardPin,  1);
        gpio_put(backwardPin, 0);
        setDuty(pwmPin, Speed);
    } else if (Speed < 0.0f) {
        // Achteruit (negatieve waarde → richting omdraaien)
        gpio_put(forwardPin,  0);
        gpio_put(backwardPin, 1);
        setDuty(pwmPin, -Speed);
    } else {
        Stop();
    }
}

int Motor::GetPwmPin() {
    return pwmPin;
}

void Motor::SetPwmPin(int PwmPin) {
    pwmPin = PwmPin;
    initPwmPin(pwmPin);
}

int Motor::GetForwardPin() {
    return forwardPin;
}

void Motor::SetForwardPin(int ForwardPin) {
    forwardPin = ForwardPin;
    gpio_init(forwardPin);
    gpio_set_dir(forwardPin, GPIO_OUT);
    gpio_put(forwardPin, 0);
}

int Motor::GetBackwardPin() {
    return backwardPin;
}

void Motor::SetBackwardPin(int BackwardPin) {
    backwardPin = BackwardPin;
    gpio_init(backwardPin);
    gpio_set_dir(backwardPin, GPIO_OUT);
    gpio_put(backwardPin, 0);
}

// ── Stop ─────────────────────────────────────────────────────────
void Motor::Stop() {
    speed = 0.0f;
    gpio_put(forwardPin,  0);
    gpio_put(backwardPin, 0);
    setDuty(pwmPin, 0.0f);
}