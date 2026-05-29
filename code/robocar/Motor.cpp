#include "Motor.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// PWM wrap value determines resolution. 65535 gives 16-bit resolution at the
// clock divider below, which results in ~490 Hz PWM — well within the range
// that avoids audible whine and keeps motor driver switching losses low.
static constexpr float PWM_CLOCK_DIV = 1.907f;
static constexpr uint  PWM_WRAP      = 65535;

// PWM input range: the motor driver accepts 0–255 (matching an 8-bit DAC
// convention used throughout the codebase), which we map linearly to 0–65535.
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

    // Scale 0–255 linearly to 0–65535.
    auto level = static_cast<uint16_t>(PWM_WRAP * pwm / PWM_INPUT_MAX);
    pwm_set_gpio_level(pin, level);
}

void Motor::SetSpeed(float newSpeed)
{
    speed = newSpeed;

    if (newSpeed > 0.0f) {
        gpio_put(forwardPin,  1);
        gpio_put(backwardPin, 0);
        SetDutyCycle(pwmPin,  newSpeed);
    } else if (newSpeed < 0.0f) {
        gpio_put(forwardPin,  0);
        gpio_put(backwardPin, 1);
        SetDutyCycle(pwmPin, -newSpeed);
    } else {
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

float Motor::GetSpeed()        const { return speed;       }
int   Motor::GetPwmPin()       const { return pwmPin;      }
int   Motor::GetForwardPin()   const { return forwardPin;  }
int   Motor::GetBackwardPin()  const { return backwardPin; }

void Motor::SetPwmPin(int pin)
{
    pwmPin = pin;
    InitPwmPin(pwmPin);
}

void Motor::SetForwardPin(int pin)
{
    forwardPin = pin;
    gpio_init(forwardPin);
    gpio_set_dir(forwardPin, GPIO_OUT);
    gpio_put(forwardPin, 0);
}

void Motor::SetBackwardPin(int pin)
{
    backwardPin = pin;
    gpio_init(backwardPin);
    gpio_set_dir(backwardPin, GPIO_OUT);
    gpio_put(backwardPin, 0);
}