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
 * @param orientation Orientation of the vehicle in radians
 * 0 radians is pointing in the positive x direction
 *
 */
struct Pose {
  Position position;
  double orientation = 0.0;
  Pose() {}
  Pose(Position position, double orientation) : position(position), orientation(orientation) {}
  Pose(double x, double y, double theta) : position(x, y), orientation(theta) {}
};

/**
 * @brief Struct for state of the vehicle
 *
 * @param pose Pose of the vehicle
 * @param last_update Timestamp of last update
 *
 */
struct VehicleState {
  Pose pose;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_update;
};

/**
 * @brief Struct for data retrieved by the IMU
 *
 * @param translational_velocity Translational velocity of the vehicle
 * @param rotational_velocity Rotational velocity of the vehicle
 * @param last_update Timestamp of last update
 * @param delta_time Time since last update
 *
 */
struct ImuUpdate {
  double translational_velocity_x = 0.0; /**< Meters per sec */
  double translational_velocity_y = 0.0; /**< Meters per sec */
  double translational_velocity = 0.0;   /**< Meters per sec */
  double rotational_velocity = 0.0;      /**< Degrees per sec */
  std::chrono::time_point<std::chrono::high_resolution_clock>
      last_update; /**< Timestamp of last update */
};

/**
 * @brief Struct for map
 *
 * @param map Map of the environment
 * @param last_update Timestamp of last update
 *
 */
struct Map {
  std::map<Position, colors::Color> map;
  std::chrono::time_point<std::chrono::high_resolution_clock>
      last_update; /**< Timestamp of last update */
};

#endif  // SRC_LOC_MAP_INCLUDE_LOC_MAP_DATA_STRUCTURES_HPP_