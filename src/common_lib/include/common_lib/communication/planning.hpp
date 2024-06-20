#include "custom_interfaces/msg/path_point.hpp"
#include "custom_interfaces/msg/path_point_array.hpp"
#include "common_lib/structures/path_point.hpp"

namespace common_lib::communication {

/**
 * @brief Converts a Path Point Array from Custom Interfaces into a vector of
 * common_lib path points
 * @param path_point_array Path Point Array from Custom Interfaces
 * @return std::vector<common_lib::structures::PathPoint>
 */ 
std::vector<common_lib::structures::PathPoint> path_point_array_from_ci_vector(
    const custom_interfaces::msg::PathPointArray &path_point_array);

}