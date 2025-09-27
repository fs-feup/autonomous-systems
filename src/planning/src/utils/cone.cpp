#include "../../include/utils/cone.hpp"

int find_cone(std::vector<std::shared_ptr<Cone>>& cones, double x, double y) {
    for (int i = 0; i < static_cast<int>(cones.size()); i++) {
        if (cones[i]->position.x == x && cones[i]->position.y == y) {
            return i;
        }
    }
    return -1;
}

int find_cone_temp(std::vector<Cone> &cones, double x, double y) {
  for (int i = 0; i < static_cast<int>(cones.size()); i++)
    if (cones[i].position.x == x && cones[i].position.y == y)
        return i;
  return -1;
}