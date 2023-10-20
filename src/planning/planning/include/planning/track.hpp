#ifndef SRC_PLANNING_PLANNING_INCLUDE_PLANNING_TRACK_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_PLANNING_TRACK_HPP_

#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "utils/cone.hpp"

/**
 * Track class. Contains the track information data and the cones position
 */

class Track {
  bool completed;
  std::vector<Cone*> leftCones;   // left side cone list
  std::vector<Cone*> rightCones;  // right side cone list

  int rightCount = 0;
  int leftCount = 0;

 public:
  /**
   * @brief Default constructor for the Track class.
   *
   * Initializes an instance of the Track class with initial values (completed = false).
   */
  Track();
  /**
   * @brief Destructor for the Track class.
   *
   * Frees memory by deleting all cone objects in the left and right cone lists.
   */
  ~Track();

  /**
   * @brief Get the right cone at a specified index.
   *
   * @param index The index of the cone to retrieve.
   * @return A pointer to the right cone at the specified index.
   */
  Cone* getRightConeAt(int index);
  /**
   * @brief Get the left cone at a specified index.
   *
   * @param index The index of the cone to retrieve.
   * @return A pointer to the left cone at the specified index.
   */
  Cone* getLeftConeAt(int index);

  /**
   * @brief Get the number of right cones.
   *
   * @return The number of right cones in the track.
   */
  int getRightConesSize();
  /**
   * @brief Get the number of left cones.
   *
   * @return The number of left cones in the track.
   */
  int getLeftConesSize();

  /**
   * @brief Fill the track data from a file.
   *
   * @param path The path to the file containing track data.
   */
  void fillTrack(const std::string& path, const std::string& testname = "");

  /**
   * @brief Add a cone to the track if the color is appropriate.
   *
   * @param xValue The x-coordinate of the cone.
   * @param yValue The y-coordinate of the cone.
   * @param color The color of the cone.
   */
  void addCone(float xValue, float yValue, const std::string& color);
  // adds the position of two cones to the track list

  /**
   * @brief Change the position of a cone based on the id of the parameter cone or creates a new
   * cone in case the id does not exist.
   *
   * @param cone A pointer to the cone to set.
   */

  void setCone(Cone* cone);
  /**
   * @brief Find a cone by its ID.
   *
   * @param id The ID of the cone to find.
   * @return A pointer to the found cone or nullptr if not found.
   */
  Cone* findCone(int id);
  /**
   * @brief Find a cone by its position.
   *
   * @param x The x-coordinate of the cone.
   * @param y The y-coordinate of the cone.
   * @return A pointer to the found cone or nullptr if not found.
   */
  Cone* findCone(float x, float y);
  /**
   * @brief Check if the direction of two cones is consistent.
   *
   * @param c1 The first cone.
   * @param c2 The second cone.
   * @param prev_vx The previous x-direction.
   * @param prev_vy The previous y-direction.
   * @return True if the direction is consistent, false otherwise.
   */
  bool vector_direction(Cone* c1, Cone* c2, float prev_vx, float prev_vy);
  /**
   * @brief Validate the cones in the track by deleting outliers.
   *
   * @return The total number of deleted outliers.
   */
  int validateCones();
  /**
  
 * @brief Delete outliers from the track.
 *
 * @param side Whether to delete outliers from the left (true) or right (false) cone list.
 * @param distance_threshold The distance threshold for identifying outliers.
 * @param order The order for fitting the spline.
 * @param coeffs_ratio The ratio of coefficients to cone count for fitting the spline.
 * @param writing Whether it's pretended to write splines or removed cones information to a file
 * @return The number of deleted outliers.
 * @details The `deleteOutliers` function is designed to remove the cone outliers from the track.
 * Outliers are data points that significantly deviate from the expected pattern or behavior of the track.
 *
 * The function operates based on the provided parameters:
 *
 * - `side`: When `side` is set to `true`, outliers are removed from the left side
 *   of the cone list; when set to `false`, outliers are removed from the right side.
 * - `distance_threshold`: This parameter determines the maximum allowable distance
 *   for a point to be considered an outlier. Points exceeding this distance from
 *   the expected behavior are identified as outliers.
 * - `order`: The order parameter influences the degree of the B-spline used for
 *   modeling the track. A higher order generally allows for more flexible curve fitting.
 * - `coeffs_ratio`: This parameter controls the complexity of the B-spline fitting
 *   process by specifying the ratio of coefficients to the total cone count.
 *
 * The function combines B-spline modeling, linear regression, and outlier detection
 * techniques to identify and remove outliers effectively. The specific implementation
 * details, such as how B-splines are created and evaluated, how linear regression is
 * applied, and how the outlier detection process works, are encapsulated within the
 * function's logic.
 *
 * @return The function returns the count of outliers that were successfully removed
 * from the track.
 */
  int deleteOutliers(bool side, float distance_threshold,
    int order, float coeffs_ratio, bool writing);

  /**
   * @brief Reset the track by clearing cone lists.
   */
  void reset();
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_TRACK_HPP_
