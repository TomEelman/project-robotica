#ifndef MOTOR_H
#define MOTOR_H

class Motor {
public:
    Motor(int pwmPin, int forwardPin, int backwardPin);

    void SetSpeed(float speed);
    void Stop();

private:
    int   pwmPin;
    int   forwardPin;
    int   backwardPin;
    float speed;

    void InitPwmPin(int pin);
    void SetDutyCycle(int pin, float pwm);
};
#endif