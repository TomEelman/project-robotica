#ifndef ENCODER_H
#define ENCODER_H
#include "pico/stdlib.h"

class Encoder {

private:
    int GPIO;
    int GPIOPinRes;
    float LinearVelocity;
    float DistanceMm;        // nieuw: voor afstand bijhouden
    int pulsesWithResolution;
    bool Updated;
    uint64_t lastTime;
    int lastPulseCount;
    uint64_t lastPulseTime;

public:
    Encoder(int GPIOPin, int GPIOPinRes);

    bool Update();
    void Reset();

    float GetLinVelocity() const;
    float GetDistanceMm()  const;   // nieuw
    int   GetGpio()        const;
    int   GetGpioPinRes()  const;
};

#endif