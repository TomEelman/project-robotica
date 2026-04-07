#ifndef IMU_H
#define IMU_H

class IMU {

private:
	int sdaPin;
	int sclPin;
	float linearVelocity;
	float angularVelocity;
	int communicationAdress;

	float currentYaw;
	bool calibrationStatus;

	bool Updated;

public:
	IMU(int SDAPin, int SCLPin, int ComAdress);

	bool Update();

	float GetLinVelocity()const;

	float GetAngVelocity()const;

	float GetCurrentYaw()const;

	bool GetCalibrationStatus()const;

	void SetComAdress(int Adres);

	int GetComAdress()const;

	int GetSDAPin()const;

	int GetSCLPin()const;
};

#endif
