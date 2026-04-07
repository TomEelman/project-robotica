#ifndef DRIVECOMMAND_H
#define DRIVECOMMAND_H

class DriveCommand {

private:
    float linearVelocity;
    float angularVelocity;

public:
    DriveCommand(float L, float A);

    float GetLinVelocity() const;

    float GetAngVelocity() const;
};

#endif