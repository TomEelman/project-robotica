#ifndef MOTOR_H
#define MOTOR_H

class Motor {

private:
    int pwmPin;
    int forwardPin;
    int backwardPin;
    float speed;

public:
    Motor(int PWMPin, int ForwardPin, int BackwardPin);

    void SetSpeed(float Speed);
    float GetSpeed();

    int GetPwmPin();
    void SetPwmPin(int PwmPin);

    int GetForwardPin();
    void SetForwardPin(int ForwardPin);

    int GetBackwardPin();
    void SetBackwardPin(int BackwardPin);

    void Stop();

};

#endif