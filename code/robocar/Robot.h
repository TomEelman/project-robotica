#ifndef ROBOT_H
#define ROBOT_H

#include "Motor.h"
#include "SensorHub.h"
#include "Drive.h"
#include "DriveCommand.h"
#include "Localisation.h"

class Robot {
public:
    Robot();

    void UpdateSensors();
    void Execute(const DriveCommand& command);

    float GetAngVelocity() const;
    float GetCurrentYaw()  const;

    SensorHub& GetSensorHub() { return sensorHub; }

private:
    Motor     motorLeft;
    Motor     motorRight;
    SensorHub sensorHub;
    Drive     drive;
    Localisation localisation;
};

#endif