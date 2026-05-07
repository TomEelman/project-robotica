#ifndef MOTOR_H
#define MOTOR_H

class Motor {
public:
    Motor(int pwmPin, int forwardPin, int backwardPin);

    void  SetSpeed(float speed);
    float GetSpeed() const;

    void Stop();

    int GetPwmPin()      const;
    int GetForwardPin()  const;
    int GetBackwardPin() const;

    void SetPwmPin     (int pin);
    void SetForwardPin (int pin);
    void SetBackwardPin(int pin);

private:
    int   pwmPin;
    int   forwardPin;
    int   backwardPin;
    float speed;

    void InitPwmPin(int pin);
    void SetDutyCycle(int pin, float pwm);
};
#endif