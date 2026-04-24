#include "Position.h"

Position::Position(float x, float y, float theta)
    : x(x), y(y), theta(theta)
{
}

float Position::GetX()     const { return x; }
float Position::GetY()     const { return y; }
float Position::GetTheta() const { return theta; }