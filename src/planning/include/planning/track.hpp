#ifndef SRC_PLANNING_INCLUDE_PLANNING_TRACK_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_TRACK_HPP_

#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "utils/cone.hpp"
#include "utils/files.hpp"

/**
 * Track class. Contains the track information data and the cones position
 */

class Track {
  bool completed;
  std::vector<Cone *> leftCones;   // left side cone list
  std::vector<Cone *> rightCones;  // right side cone list

  int rightCount = 0;
  int leftCount = 0;

 public:
  /**
   * @brief Default constructor for the Track class.
   *
   * Initializes an instance of the Track class with initial values (completed =
   * false).
   */
  Track();
  /**
   * @brief Destructor for the Track class.
   *
   * Frees memory by deleting all cone objects in the left and right cone lists.
   */
  ~Track();

  /**
   * @brief log cones on one side of the track
   *
   * @param side if true -> left, false -> right
   */
  void logCones(bool side);

  /**
   * @brief used to calculate max distance between consecutive cones for testing
   *
   * @param side true -> left | false -> right
   * @return maximum distance between consecutive cones
   */
  float getMaxDistance(bool side);

  /**
   * @brief Get the right side cones.
   *
   * @return A vector of pointers to the right side cones.
   */
  std::vector<Cone *> get_right_cones() const;

  /**
   * @brief Get the left side cones.
   *
   * @return A vector of pointers to the left cones.
   */
  std::vector<Cone *> get_left_cones() const;

  /**
   * @brief Get the right cone at a specified index.
   *
   * @param index The index of the cone to retrieve.
   * @return A pointer to the right cone at the specified index.
   */
  Cone *getRightConeAt(int index);
  /**
   * @brief Get the left cone at a specified index.
   *
   * @param index The index of the cone to retrieve.
   * @return A pointer to the left cone at the specified index.
   */
  Cone *getLeftConeAt(int index);

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
  void fillTrack(const std::string &path);

  /**
   * @brief Add a cone to the track if the color is appropriate.
   *
   * @param xValue The x-coordinate of the cone.
   * @param yValue The y-coordinate of the cone.
   * @param color The color of the cone.
   */
  void addCone(float xValue, float yValue, const std::string &color);
  // adds the position of two cones to the track list

  /**
   * @brief Change the position of a cone based on the id of the parameter cone
   * or creates a new cone in case the id does not exist.
   *
   * @param cone A pointer to the cone to set.
   */

  void setCone(Cone *cone);
  /**
   * @brief Find a cone by its ID.
   *
   * @param id The ID of the cone to find.
   * @return A pointer to the found cone or nullptr if not found.
   */
  Cone *findCone(int id);
  /**
   * @brief Find a cone by its position.
   *
   * @param x The x-coordinate of the cone.
   * @param y The y-coordinate of the cone.
   * @return A pointer to the found cone or nullptr if not found.
   */
  Cone *findCone(float x, float y);

  /**
   * @brief Used for logging content of gsl_vectors
   *
   * @param gsl gsl_vector of pointers
   * @param n number of elements in the gsl_vector
   * @return content of the gsl_vector separated with spaces
   */
  std::string gsl_vectorLog(gsl_vector *gsl, int n);

  /**
   * @brief Check if the direction of two cones is consistent.
   *
   * @param c1 The first cone.
   * @param c2 The second cone.
   * @param prev_vx The previous x-direction.
   * @param prev_vy The previous y-direction.
   * @return True if the direction is consistent, false otherwise.
   */
  bool vector_direction(Cone *c1, Cone *c2, float prev_vx, float prev_vy);

  /**
   * @brief Order the cones before fitting splines
   *
   * @param unord_cone_seq Unordered array of cones
   * @return Ordered array of cones
   */
  std::vector<Cone *> orderCones(std::vector<Cone *> *unord_cone_seq);

  /**
   * @brief function to fit spline to cone sequence
   *
   * @param side Whether to fit spline to the left (true) or right (false)
   * cone list.
   * @param precision controls the amount of points in the parametrized
   * spline (1 means one point per unit of the parameter unit). As a good rule of thumb
   * use 1 for outlier removal (that will mean each cone has its corresponding spline point)
   * and 10-100 for path smoothing
   * @param order Degree of the B-spline. It is a parameter that determines
   * the degree of the polynomial functions used to interpolate between breakpoints.
   * @param coeffs_ratio The ratio of coefficients to cone count for fitting the
   * spline
   * @param cone_seq ordered sequence of cones
   * @return cones in the spline
   */
  std::vector<Cone *> fitSpline(bool side, int precision, int order, float coeffs_ratio,
                                std::vector<Cone *> cone_seq);

  /**
   * @brief Validate the cones in the track by deleting outliers.
   */
  void validateCones();
  /**

 * @brief Delete outliers from the track.
 *
 * @param side Whether to delete outliers from the left (true) or right (false)
 cone list.
 * @param distance_threshold The distance threshold for identifying outliers.
 * @param order The order for fitting the spline.
 * @param coeffs_ratio The ratio of coefficients to cone count for fitting the
 spline.
 * @param writing Whether it's intended to write splines or removed cones'
 information to a file
 * @param precision Ratio of cones in the cones to the initial number of cones
 * @details The function combines B-spline modeling, linear regression, and outlier
 detection
 * techniques to identify and remove outliers effectively. The specific
 implementation
 * details, such as how B-splines are created and evaluated, how linear
 regression is
 * applied, and how the outlier detection process works, are encapsulated within
 the function's logic.
 */
  void deleteOutliers(bool side, int order, float coeffs_ratio, bool writing, int precision);

  /**
   * @brief Reset the track by clearing cone lists.
   */
  void reset();
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_PLANNING_TRACK_HPP_
