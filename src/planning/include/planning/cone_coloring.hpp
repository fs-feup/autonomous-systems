#ifndef SRC_PLANNING_INCLUDE_PLANNING_CONE_COLORING_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_CONE_COLORING_HPP_

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include "common_lib/competition_logic/color.hpp"
#include "common_lib/maths/angle_and_norms.hpp"
#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "common_lib/structures/track_side.hpp"
#include "config/cone_coloring_config.hpp"
#include "rclcpp/rclcpp.hpp"

using Cone = common_lib::structures::Cone;
using Pose = common_lib::structures::Pose;
using Position = common_lib::structures::Position;
using TwoDVector = common_lib::structures::Position;
using TrackSide = common_lib::structures::TrackSide;
using Color = common_lib::competition_logic::Color;
using AngleAndNorms = common_lib::maths::AngleAndNorms;

class ConeColoring {
private:
  /**
   * @brief configuration of the cone coloring algorithm
   *
   */
  ConeColoringConfig config_;

  /**
   * @brief function to remove duplicates from a vector of cones
   *
   * @param cones vector of cones
   */
  void remove_duplicates(std::vector<Cone>& cones);

  /**
   * @brief calculate the expected position of the initial cones
   *
   * @param car_pose car pose in the map relative to the origin
   * @param track_side side of the track of the expected position to be calculated
   */
  Position expected_initial_cone_position(const Pose& car_pose, const TrackSide& track_side);

  /**
   * @brief function to find the initial cone according to the distance to the expected position
   *
   * @param cones set of cones
   * @param car_pose car pose in the map relative to the origin
   * @param track_side side of the track of the expected position to be calculated
   * @return Cone initial cone
   */
  Cone find_initial_cone(const std::unordered_set<Cone, std::hash<Cone>>& cones,
                         const Pose& car_pose, const TrackSide track_side);

  /**
   * @brief calculate the position of a virtual cone relative to a real one
   *
   * @param initial_cone cone to be used as reference
   * @param car_pose pose of the car
   * @return virtual cone
   */
  Cone virtual_cone_from_initial_cone(const Cone& initial_cone, const Pose& car_pose);

  /**
   * @brief function to place the initial cones, including the virtual ones
   *
   * @param uncolored_cones set of cones
   * @param colored_blue_cones vector of blue cones (where the blue cones will be added)
   * @param colored_yellow_cones vector of yellow cones (where the yellow cones will be added)
   * @param car_pose car pose in the map relative to the origin
   * @param n_colored_cones number of colored cones which will be updated
   */
  void place_initial_cones(std::unordered_set<Cone, std::hash<Cone>>& uncolored_cones,
                           std::vector<Cone>& colored_blue_cones,
                           std::vector<Cone>& colored_yellow_cones, const Pose& car_pose,
                           int& n_colored_cones);

  /**
   * @brief calculate the cost of coloring a cone
   *
   * @param next_cone potential cone to be colored
   * @param last_cone last cone to be colored in the side of interest
   * @param previous_to_last_vector vector formed by the last two cones in the side of interest
   * @param colored_to_input_cones_ratio ration of cones which have been colored
   * @return double cost
   */
  double calculate_cost(const Cone& next_cone, const Cone& last_cone,
                        const TwoDVector& previous_to_last_vector,
                        const double& colored_to_input_cones_ratio);

  /**
   * @brief select the next cone (which minimizes the cost) to be colored if its cost is less than
   * the maximum established in the configuration
   *
   * @param uncolored_cones input cones
   * @param colored_cones vector to add the next cone
   * @param n_colored_cones number of cones which have been colored (updated in the function)
   * @param n_input_cones number of input cones (used to calculate the ratio of colored cones to
   * input cones)
   * @return true if the cone has been successfully colored
   * @return false if all cones had a cost higher than the maximum established or were too far away
   */
  bool try_to_color_next_cone(std::unordered_set<Cone, std::hash<Cone>>& uncolored_cones,
                              std::vector<Cone>& colored_cones, int& n_colored_cones,
                              const int n_input_cones);

public:
  /**
   * @brief Construct a new default ConeColoring object
   *
   */
  ConeColoring() = default;
  /**
   * @brief Construct a new ConeColoring object with a given configuration
   *
   */
  explicit ConeColoring(ConeColoringConfig config) : config_(config){};
  /**
   * @brief color all cones
   *
   * @param cones vector of input cones without color
   * @param car_pose pose of the car
   * @return std::pair<std::vector<Cone>, std::vector<Cone>> the first vector contains the blue
   * cones and the second vector contains the yellow cones
   */
  std::pair<std::vector<Cone>, std::vector<Cone>> color_cones(std::vector<Cone> cones,
                                                              const Pose& car_pose);

  /**
   * @brief tests are declared as friend to test the behaviour of private functions
   *
   */
  friend class ConeColoring_place_first_cones1_Test;
  friend class ConeColoring_place_first_cones2_Test;
  friend class ConeColoring_fullconecoloring1_Test;
  friend class ConeColoring_fullconecoloring2_Test;
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_CONE_COLORING_HPP_