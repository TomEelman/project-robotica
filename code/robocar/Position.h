#ifndef POSITION_H
#define POSITION_H

class Position {
private:
    float x;
    float y;
    float theta;

public:
    Position(float x = 0.0f, float y = 0.0f, float theta = 0.0f);

    float GetX()     const;
    float GetY()     const;
    float GetTheta() const;

    void SetX(float v);
    void SetY(float v);
    void SetTheta(float v);
};

#endif