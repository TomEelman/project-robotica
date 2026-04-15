#ifndef ROBOT_H
#define ROBOT_H

#include "Motor.h"
#include "SensorHub.h"
#include "Drive.h"
#include "DriveCommand.h"
#include "Kalmanfilter.h"

class Robot {

private:
    Motor motorLeft;
    Motor motorRight;
    SensorHub sensorHub;
    Drive drive;
    KalmanFilter kalmanLinks;   // ← hier toevoegen
    KalmanFilter kalmanRechts; 

public:
    Robot();

    void Update();
};

#endif