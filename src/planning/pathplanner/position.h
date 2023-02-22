

#include "cmath"
#include "vector"
/**
 * Position class. Stores the x and y values of the car
*/
class Position {
    float x, y;

public:
    Position(float x, float y);

    float getX() const;

    void setX(float x);

    float getY() const;

    void setY(float y);

    /**
     * Euclidean distance to another position
    */
    float getDistanceTo(Position* dest);

};


