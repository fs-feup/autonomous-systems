#ifndef SRC_LOC_MAP_INCLUDE_LOC_MAP_DATA_STRUCTURES_HPP_
#define SRC_LOC_MAP_INCLUDE_LOC_MAP_DATA_STRUCTURES_HPP_

#include <chrono>
#include <map>
#include <string>

#include "utils/color.hpp"
#include "utils/position.hpp"

/**
 * @brief Struct for localization
 *
 * @param position Position of the vehicle
 * @param orientation Orientation of the vehicle
 *
 */
struct Pose {
  Position position;
  double orientation = 0.0;
};

/**
 * @brief Struct for state of the vehicle
 *
 * @param pose Pose of the vehicle
 * @param translational_velocity_x Translational velocity of the vehicle in x direction
 * @param translational_velocity_y Translational velocity of the vehicle in y direction
 * @param rotational_velocity_x Rotational velocity of the vehicle in x axis
 * @param rotational_velocity_y Rotational velocity of the vehicle in y axis
 *
 */
struct VehicleState {
  Pose pose;
  float translational_velocity_x = 0.0;
  float translational_velocity_y = 0.0;
  float rotational_velocity_x = 0.0;
  float rotational_velocity_y = 0.0;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_update;
};

/**
 * @brief Struct for map
 *
 * @param map Map of the environment
 *
 */
struct Map {
  std::map<Position, colors::Color> map;
};

#endif  // SRC_LOC_MAP_INCLUDE_LOC_MAP_DATA_STRUCTURES_HPP_