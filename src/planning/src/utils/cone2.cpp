#include "../../include/utils/cone2.hpp"

int find_cone(std::vector<Cone> &cones, double x, double y){
    for (int i = 0; i < cones.size(); i++)
        if (cones[i].position.x == x && cones[i].position.y == y)
            return i;
    return -1;
}
