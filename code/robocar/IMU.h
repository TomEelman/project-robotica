#ifndef IMU_H
#define IMU_H

class IMU {

private:
	int GPIO;
	float LinearVelocity;
	float Angularvelocity;
	int CommunicationAdress;
	int CurrentYaw;
	bool CalibrationStatus;

public:
	IMU(int GPIOPin);

	/**
	 * velocity is een struct
	 */
	velocity Update();
};

#endif
