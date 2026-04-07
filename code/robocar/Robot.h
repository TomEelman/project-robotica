#ifndef ROBOT_H
#define ROBOT_H

#include "Motor.h"
#include "SensorHub.h"
#include "Drive.h"
#include "DriveCommand.h"

class Robot {

private:
    Motor motorLeft;
    Motor motorRight;
    SensorHub sensorHub;
    Drive drive;

public:
    Robot();

    void Update();
};

#endif