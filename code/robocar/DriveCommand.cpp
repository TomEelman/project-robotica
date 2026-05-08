#include "DriveCommand.h"

DriveCommand::DriveCommand(float linearVelocity, float angularVelocity)
    : linearVelocity(linearVelocity),
      angularVelocity(angularVelocity)
{
}

float DriveCommand::GetLinVelocity() const { return linearVelocity;  }
float DriveCommand::GetAngVelocity() const { return angularVelocity; }