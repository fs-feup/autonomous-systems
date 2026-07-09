#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "common_lib/structures/cone.hpp"
#include "io/output/output_snapshot.hpp"
#include "track/track.hpp"

/**
 * @brief Axis-aligned car collision rectangle in the car frame.
 */
struct Hitbox {
  double center_x = 0.0;
  double center_y = 0.0;
  double half_length = 0.0;
  double half_width = 0.0;
};

/**
 * @brief Tracks cone collisions and lap cone penalties.
 */
class CollisionStatistics {
public:
  /**
   * @brief Load cone collision settings and car hitboxes.
   */
  CollisionStatistics(const Track& track, const std::string& discipline,
                      const std::string& car_parameters_config);

  /**
   * @brief Clear cone hits for a new lap.
   */
  void reset_lap(StatisticsSnapshot& snapshot);

  /**
   * @brief Check current vehicle pose against all unhit cones.
   */
  void update(const VehicleModelSnapshot& vehicle_snapshot, bool lap_timing_started,
              StatisticsSnapshot& snapshot);

  /**
   * @brief Penalty time from cones hit in the current lap.
   */
  double current_lap_penalty_time() const;

  /**
   * @brief Cones hit during the current lap.
   */
  std::vector<common_lib::structures::Cone> recently_hit_cones() const;

private:
  // Track and car collision geometry.
  std::vector<common_lib::structures::Cone> cones_;
  std::vector<Hitbox> car_hitboxes_;
  double standard_cone_radius_m_ = 0.115;
  double large_cone_radius_m_ = 0.15;

  // Current lap penalty state.
  double default_cone_penalty_time_s_ = 2.0;
  double skidpad_cone_penalty_time_s_ = 0.2;
  std::vector<bool> cone_was_hit_;
  std::size_t hit_cone_count_ = 0;
  double penalty_time_per_cone_ = 0.0;

  // Configuration helpers.
  void load_cone_collision_config();
  void load_penalty_config();
  double penalty_time_for_discipline(std::string discipline) const;
  std::vector<Hitbox> load_car_hitboxes(const std::string& car_parameters_config) const;

  // Collision helpers.
  double cone_radius(const common_lib::structures::Cone& cone) const;
  bool collides_with_cone(const VehicleModelSnapshot& vehicle_snapshot,
                          const common_lib::structures::Cone& cone) const;
};
