    #ifndef ROBOT_H
    #define ROBOT_H

    #include "Motor.h"
    #include "SensorHub.h"
    #include "Drive.h"
    #include "DriveCommand.h"
    #include "Localisation.h"
    class Robot {

    private:
        Motor motorLeft;
        Motor motorRight;
        SensorHub sensorHub;
        Drive drive; 
        Localisation localisation;
    public:
        Robot();

        void Update();
    };

    #endif