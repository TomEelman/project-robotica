#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include "Path.h"
#include "Position.h"
#include "DriveCommand.h"
#include "Path.h"
class Navigator {

private:
    Path path;
    Position currentTarget;

    bool isUpdated;
    bool saved;

    float CalculateDistance(Position current, Position target);
    float CalculateAngle(Position current, Position target);

public:
    Navigator();

    void Update(Position current);

    DriveCommand GetNextCommand(Position Current);

    bool ReachedPoint(Position Current);

    bool SaveWaypoints();

    bool IsUpdated() const;
};

#endif