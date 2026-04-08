#ifndef ENCODER_H
#define ENCODER_H
#include "pico/stdlib.h"
class Encoder {

private:
	int GPIO;
	int GPIOPinRes;
	float LinearVelocity;
	int pulses;
	int pulsesWithResolution;
	bool Updaded;
	    uint64_t lastTime;
    int lastPulseCount;
    uint64_t lastPulseTime;

public:
	Encoder(int GPIOPin, int GPIOPinRes);

	bool Update();

	void Reset();

	int GetPulses() const;

	int GetPulsesWR()const;

	float GetLinVelocity()const;

	int GetGpio()const;

	int GetGpioPinRes()const;
};

#endif
