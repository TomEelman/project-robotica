#ifndef DRIVECOMMAND_H
#define DRIVECOMMAND_H

class DriveCommand {
public:
    DriveCommand(float linearVelocity, float angularVelocity);

    float GetLinVelocity()  const;
    float GetAngVelocity()  const;

private:
    float linearVelocity;
    float angularVelocity;
};

#endif