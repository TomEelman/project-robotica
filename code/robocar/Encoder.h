#ifndef ENCODER_H
#define ENCODER_H

class Encoder {

private:
	int GPIO;
	float LinearVelocity;
	int pulses;
	int pulsesWithResolution;

public:
	Encoder(int GPIOPin);

	/**
	 * velocity is een struct
	 */
	velocity Update();

	void Reset();

	void GetEncoderData();
};

#endif
