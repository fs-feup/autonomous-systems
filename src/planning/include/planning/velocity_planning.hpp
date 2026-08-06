#pragma once

#include <cmath>
#include <numeric>

#include "common_lib/structures/path_point.hpp"
#include "common_lib/structures/pose.hpp"
#include "config/velocity_config.hpp"

using PathPoint = common_lib::structures::PathPoint;
using Pose = common_lib::structures::Pose;

/**
 * @brief Represents a contiguous section of the path between two corner apexes.
 *
 * Sections are straight-centered: each section spans from one curvature peak
 * (corner apex) to the next. This mirrors the natural racing-line atom of
 * braking zone → straight → acceleration zone.
 *
 * Limit adaptation is tracked per section via a rolling mean of cross-track
 * error samples. Once enough samples have been collected the per-node
 * acceleration/lateral limits for the entire section are nudged up or down
 * and the accumulator is reset.
 */
struct Section {
  int start_idx;
  int end_idx;
  double mean_error;
  int sample_count;
  double current_long_acc;
  double current_lat_acc;
  double max_error;
  double min_error;
  bool is_corner;
};


/**
 * @brief Computes velocity profiles for a planned path based on curvature and dynamics constraints.
 *
 * The VelocityPlanning class generates a velocity profile along a path by:
 * - Estimating curvature using the Menger curvature formula (circle fitting through three points),
 * - Deriving maximum allowable velocities from lateral acceleration limits,
 * - Applying a friction ellipse model to account for combined longitudinal and lateral tire forces,
 * - Propagating acceleration constraints forward and braking constraints backward along the path.
 *
 * The velocity planner respects the friction circle constraint: a_x² + a_y² ≤ a_max²,
 * ensuring that the vehicle stays within tire grip limits during combined cornering and
 * acceleration/braking.
 *
 * Per-section limit adaptation:
 * - On path set, the path is divided into sections at curvature peaks (corner apexes).
 * - Each call to adapt_limits() accumulates cross-track error for the section the
 *   vehicle currently occupies.
 * - After section_adapt_samples_ samples the mean error is evaluated and every node
 *   in the section has its acceleration limits adjusted according to the same error
 *   bands used previously, then the accumulator resets.
 */
class VelocityPlanning {
public:
  VelocityPlanning() = default;
  explicit VelocityPlanning(VelocityPlanningConfig config) : config_(config) {}

  /**
   * @brief Assigns a velocity to each point of the path and computes sections.
   *
   * Sections are (re)computed here. Call this whenever the path changes.
   *
   * @param final_path Vector of path points to update with planned velocities.
   */
  void set_velocity(std::vector<PathPoint> &final_path);

  /**
   * @brief Computes velocity for trackdrive scenarios.
   *
   * @param final_path Vector of path points to update with planned velocities.
   */
  void trackdrive_velocity(std::vector<PathPoint> &final_path);

  /**
   * @brief Applies a braking velocity profile starting after a given braking distance.
   *
   * @param final_path Vector of path points to update with planned velocities.
   * @param braking_distance Distance along the path before braking begins.
   */
  void stop(std::vector<PathPoint> &final_path, double braking_distance);

  /**
   * @brief Accumulates cross-track error for the section the vehicle currently
   *        occupies and, once enough samples have been collected, adjusts the
   *        per-node acceleration limits for that entire section.
   *
   * Sections must have been computed first via set_velocity() or
   * trackdrive_velocity().
   *
   * @param pose  Current vehicle pose.
   * @param path  The active path (same one passed to set_velocity).
   */
  void adapt_limits(Pose &pose, std::vector<PathPoint> &path, bool is_closed);
  const std::vector<Section> &get_sections() const { return sections_; }

private:
  VelocityPlanningConfig config_;

  static constexpr double epsilon = 1e-9;

  /// Path sections computed once per set_velocity() call.
  std::vector<Section> sections_;

  int current_section_idx_ = -1;

  /// Minimum curvature value for a point to be considered a corner apex / section boundary.
  /// Exposed here so it can be tuned; consider adding to VelocityPlanningConfig.
  double curvature_peak_threshold_{0.05};

  /// Minimum number of path points between two section boundaries (prevents over-segmentation).
  int min_section_spacing_{5};

  // -----------------------------------------------------------------------
  // Internal helpers
  // -----------------------------------------------------------------------

  double find_curvature(const PathPoint &p1, const PathPoint &p2, const PathPoint &p3);

  void point_speed(const std::vector<double> &curvatures, std::vector<double> &velocities);

  void acceleration_limiter(const std::vector<PathPoint> &points, std::vector<double> &velocities,
                            const std::vector<double> &curvatures);

  void braking_limiter(std::vector<PathPoint> &points, std::vector<double> &velocities,
                       const std::vector<double> &curvatures);

  //TODO: CHANGE DOCS
  void compute_sections(const std::vector<double> &curvatures, bool is_closed);

  /**
   * @brief Returns the index of the section that contains path index point_idx,
   *        or -1 if no section covers it.
   */
  int find_section(int point_idx) const;

  /**
   * @brief Applies a limit delta to every node inside a section.
   *
   * @param section_idx  Index into sections_.
   * @param longitudinal_acc  Delta added to every node's longitudinal limit.
   * @param lateral_acc       Delta added to every node's lateral limit.
   */
  void change_section_limits(int section_idx, double longitudinal_acc, double lateral_acc);

  double get_pose_error(const Pose &pose, const std::vector<PathPoint> &path, size_t &best_index);
};