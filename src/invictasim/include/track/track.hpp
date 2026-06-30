#pragma once

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <utility>
#include <vector>

#include "common_lib/competition_logic/color.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/position.hpp"

/**
 * @brief The Track class is responsible for loading track information from a YAML file and
 * providing access to the cones on the track and the starting position for the vehicle.
 */
class Track {
public:
  /**
   * @brief Construct a new Track object by loading track information from a YAML file.
   */
  explicit Track(std::string file_name);

  /**
   * @brief Get the track information, which consists of a vector of Cone objects. Each Cone
   * contains the position and color of a cone on the track.
   * @return std::vector<common_lib::structures::Cone> Vector of Cone objects
   */
  const std::vector<common_lib::structures::Cone>& get_cones() const;

  /**
   * @brief Get the starting position for the vehicle on the specified track.
   * @return common_lib::structures::Position Starting position for the vehicle.
   */
  common_lib::structures::Position get_start_position() const;

  /**
   * @brief Get the timing line endpoints for lap counting.
   * @return Pair with the two endpoints of the timing line.
   */
  std::pair<common_lib::structures::Position, common_lib::structures::Position> get_timing_line()
      const;

  /**
   * @brief Get all configured timing lines. The first line is the default timing line.
   */
  const std::vector<std::pair<common_lib::structures::Position, common_lib::structures::Position>>&
  get_timing_lines() const;

private:
  std::vector<common_lib::structures::Cone> cones_;  ///< List of cones on the track
  common_lib::structures::Position start_position_;  ///< Starting position
  std::vector<std::pair<common_lib::structures::Position, common_lib::structures::Position>>
      timing_lines_;  ///< Timing line endpoints

  /**
   * @brief Adjust timing line endpoints to the local track width near the configured line.
   */
  void fit_timing_line_to_track_width();
};
