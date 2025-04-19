#pragma once

#include <cone_validator/cone_validator.hpp>
#include <utils/cluster.hpp>

/**
 * @brief The HeightValidator class is responsible for validating cones based on their height.
 *
 * This class inherits from the ConeValidator class and overrides the coneValidator method
 * to provide height-based validation logic.
 */
class HeightValidator : public ConeValidator {
private:
  double _min_height_;        ///< Min Height threshold for cone validation */
  double _large_max_height_;  ///< Max Height threshold for large cones */
  double _small_max_height_;  ///< Max Height treshhold for small cones */
  double _height_cap_;        ///< Minimum ratio result needed for return values to not be 0. */

public:
  /**
   * @brief Constructs a new HeightValidator object with the specified height threshold.
   *
   * @param min_height Min Height threshold for cone validation
   * @param max_height Max Height threshold for cone validation
   * @param small_max_height Max Height treshhold for small cones
   * @param height_cap Minimum ratio result needed for return values to not be 0
   *
   */
  explicit HeightValidator(double min_height, double large_max_height, double small_max_height,
                           double height_cap);

  /**
   * @brief Validates a cone based on its height relative to a plane.
   *
   * This method overrides the coneValidator method of the base class ConeValidator.
   * It validates whether the given cone, represented by a point cloud cluster,
   * meets the height criteria with respect to the provided plane.
   *
   * @param cone_point_cloud Pointer to a Cluster object representing the point cloud of the cone.
   * @param plane The plane against which the cone's height is evaluated.
   * @return vector containing:
   * Index 0 -> the ratio between the clusters height and the limit, 1 if inside.
   * Index 1 -> if in height interval, how close is it to the maximum height, else 0.
   */
  void coneValidator(Cluster* cone_point_cloud, EvaluatorResults* results,
                     Plane& plane) const override;

  /**
   * @brief Virtual destructor for HeightValidator.
   */
  virtual ~HeightValidator() = default;
};
