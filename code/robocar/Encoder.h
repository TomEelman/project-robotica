#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"

class Encoder {
public:
    Encoder(int gpioPin, int gpioPinRes);

    // Returns true when a new velocity sample was produced (once per UPDATE_INTERVAL_US).
    bool Update();
    void Reset();

    float GetLinVelocity() const;
    float GetDistanceMm()  const;
    int   GetGpio()        const;
    int   GetGpioPinRes()  const;

    // True if Update() produced a new sample since the last ConsumeFreshFlag() call.
    // Used by Drive to avoid re-running PID on stale encoder data.
    bool HasFreshData()     const { return freshData; }
    void ConsumeFreshFlag()       { freshData = false; }

private:
    int      gpio;
    int      gpioPinRes;
    int      pulsesPerRotation;

    float    linearVelocity;
    float    distanceMm;

    int      lastPulseCount;
    uint64_t lastTime;
    uint64_t lastPulseTime;

    bool     updated;
    bool     freshData;
};

#endif