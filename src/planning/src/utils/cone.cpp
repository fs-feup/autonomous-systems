#include "../../include/utils/cone.hpp"

int find_cone(const std::vector<Cone> &cones, double x, double y) {
  for (int i = 0; i < static_cast<int>(cones.size()); i++)
    if (cones[i].position.x == x && cones[i].position.y == y) return i;
  return -1;
}
