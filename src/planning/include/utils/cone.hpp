#include <vector>
#include "common_lib/structures/cone.hpp"

using Cone = common_lib::structures::Cone;

int find_cone(std::vector<std::shared_ptr<Cone>>& cones, double x, double y);

int find_cone_temp(std::vector<Cone> &cones, double x, double y); 
