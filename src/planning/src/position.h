

#include "cmath"
#include "vector"

class Position {
    float x, y;

public:
    Position(float x, float y);

    float getX() const;

    void setX(float x);

    float getY() const;

    void setY(float y);

    float getDistanceTo(Position* dest);

};


