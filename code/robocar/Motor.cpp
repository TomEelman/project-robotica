#include "Motor.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"


static constexpr float PWM_CLOCK_DIV = 1.907f;

// 16 bits 
static constexpr uint  PWM_WRAP      = 65535;

static constexpr float PWM_INPUT_MAX = 255.0f;

Motor::Motor(int pwmPin, int forwardPin, int backwardPin)
    : pwmPin(pwmPin),
      forwardPin(forwardPin),
      backwardPin(backwardPin),
      speed(0.0f)
{
    gpio_init(forwardPin);
    gpio_set_dir(forwardPin, GPIO_OUT);
    gpio_put(forwardPin, 0);

    gpio_init(backwardPin);
    gpio_set_dir(backwardPin, GPIO_OUT);
    gpio_put(backwardPin, 0);

    InitPwmPin(pwmPin);
    SetDutyCycle(pwmPin, 0.0f);
}

void Motor::InitPwmPin(int pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint       slice = pwm_gpio_to_slice_num(pin);
    pwm_config cfg   = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, PWM_CLOCK_DIV);
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(slice, &cfg, true);
}

void Motor::SetDutyCycle(int pin, float pwm)
{
    if (pwm < -PWM_INPUT_MAX) pwm = -PWM_INPUT_MAX;
    if (pwm >  PWM_INPUT_MAX) pwm =  PWM_INPUT_MAX;

    // Scale 0–255 linear to 0–65535.
    auto level = static_cast<uint16_t>(PWM_WRAP * pwm / PWM_INPUT_MAX);
    pwm_set_gpio_level(pin, level);
}

void Motor::SetSpeed(float newSpeed)
{
    speed = newSpeed;

    if (newSpeed > 0.0f) { // forward
        gpio_put(forwardPin,  1);
        gpio_put(backwardPin, 0);
        SetDutyCycle(pwmPin,  newSpeed);
    } else if (newSpeed < 0.0f) { // backward
        gpio_put(forwardPin,  0);
        gpio_put(backwardPin, 1);
        SetDutyCycle(pwmPin, -newSpeed);
    } else { // no movement
        Stop();
    }
}

void Motor::Stop()
{
    speed = 0.0f;
    gpio_put(forwardPin,  0);
    gpio_put(backwardPin, 0);
    SetDutyCycle(pwmPin, 0.0f);
}
