#include "../../include/utils/cone.hpp"

int find_cone(std::vector<Cone> &cones, double x, double y){
    for (size_t i = 0; i < cones.size(); i++)
        if (cones[i].position.x == x && cones[i].position.y == y)
            return (int)i;
    return -1;
}
