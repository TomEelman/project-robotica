#include "DriveCommand.h"

DriveCommand::DriveCommand(float L, float A)
{
    linearVelocity = L;
    angularVelocity = A;
}

float DriveCommand::GetLinVelocity() const
{
    return linearVelocity;
}

float DriveCommand::GetAngVelocity() const
{
    return angularVelocity;
}