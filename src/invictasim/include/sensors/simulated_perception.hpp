#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "common_lib/structures/cone.hpp"
#include "common_lib/structures/pose.hpp"
#include "sensors/sensors_base.hpp"

/**
 * @brief Simulates a LiDAR cone pipeline.
 *
 * Transforms ground-truth cones into the vehicle frame and applies the sensor's limits: field of
 * view, range-dependent detection probability, range-dependent position noise, and spurious
 * detections.
 */
class SimulatedPerception : public Sensor {
public:
  /**
   * @brief Construct a new SimulatedPerception object and load configuration.
   * @param config_path Path to the perception.yaml configuration file.
   */
  explicit SimulatedPerception(const std::string& config_path);

  /**
   * @brief Produce the cones this sensor would report.
   *
   * @param cones Ground-truth cones in the map frame.
   * @param vehicle_pose Current vehicle pose in the map frame.
   * @param dt Seconds since the previous call, used to make the spurious-detection rate
   *           independent of how often this is called.
   * @return Detected cones, positioned in the vehicle frame.
   */
  std::vector<common_lib::structures::Cone> perception_error(
      const std::vector<common_lib::structures::Cone>& cones,
      const common_lib::structures::Pose& vehicle_pose, double dt);

  /**
   * @brief Forget every spurious detection that is currently being held.
   */
  void reset();

private:
  /// Upper bound on retained spurious detections, so a long run cannot grow without limit.
  static constexpr std::size_t kMaxPersistentOutliers = 32;

  // Sensor geometry
  double height_ = 0.8;          ///< LiDAR height above the ground (m)
  double cone_height_ = 0.16;    ///< Height of the cone centroid the beam returns from (m)
  double max_range_ = 30.0;      ///< Range at which detection probability is 50% (m)
  double horizontal_fov_ = 0.0;  ///< Horizontal field of view (rad)
  double vertical_fov_ = 0.0;    ///< Vertical field of view (rad)
  double mounting_pitch_ = 0.0;  ///< Mounting pitch, positive nose-down (rad)

  // Detection and noise
  double detection_probability_alpha_ = 0.5;  ///< Sigmoid steepness of the range roll-off
  double noise_std_dev_base_ = 0.0;           ///< Position noise at zero range (m)
  bool noise_scales_with_range_ = true;       ///< Whether noise grows with range
  double noise_range_scaling_ = 0.0;          ///< Extra position noise per metre of range

  // Spurious detections
  double outlier_rate_hz_ = 0.0;            ///< Expected spurious detections per second
  double persistent_outlier_chance_ = 0.0;  ///< Chance a spurious detection is retained

  /// Retained spurious detections, in the map frame.
  std::vector<common_lib::structures::Cone> persistent_outliers_;

  /**
   * @brief Whether a point in the vehicle frame falls inside the sensor's field of view.
   *
   * @param x_vehicle Forward distance from the vehicle origin (m).
   * @param y_vehicle Left distance from the vehicle origin (m).
   * @return True when the cone is within both the horizontal and vertical field of view.
   */
  bool is_in_fov(double x_vehicle, double y_vehicle) const;

  /**
   * @brief Append newly generated spurious detections to a working list.
   *
   * @param vehicle_pose Current vehicle pose, used to place them in front of the car.
   * @param dt Seconds since the previous call.
   * @param cones_to_process List to append to.
   */
  void spawn_outliers(const common_lib::structures::Pose& vehicle_pose, double dt,
                      std::vector<common_lib::structures::Cone>& cones_to_process);
};
