#ifndef LIDAR_H
#define LIDAR_H
#define SCAN_SIZE 360
class LIDAR {

private:
	int port;
	int maxRange;
	int minAngle;
	int scanData[SCAN_SIZE];
	int rotationSpeed;
	int currentAngle;
	int nextAngle;
	bool Updaded;

public:
	void LiDAR(int GPIOPin);

	void Update();

	void IsObjectInRange(int minAngle, int MaxAngle, int Treshold);

	void ApplyMotionCorrection(float CurrentYaw);
};

#endif
