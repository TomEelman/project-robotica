#ifndef LIDAR_H
#define LIDAR_H

class LIDAR {

private:
	int port;
	int maxRange;
	int minAngle;
	int scanData[];
	int rotationSpeed;
	int currentAngle;
	int nextAngle;

public:
	void LiDAR(int GPIOPin);

	void Update();

	void IsObjectInRange(int minAngle, int MaxAngle, int Treshold);

	void applyMotionCorrection(float AngularVelocity, float parameter);
};

#endif
