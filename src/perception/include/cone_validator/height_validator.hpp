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
  double height;  // Height threshold for cone validation

 public:
  /**
   * @brief Constructs a new HeightValidator object with the specified height threshold.
   *
   * @param height The height threshold for cone validation.
   */
  explicit HeightValidator(double height);

  /**
   * @brief Validates a cone based on its height relative to a plane.
   *
   * This method overrides the coneValidator method of the base class ConeValidator.
   * It validates whether the given cone, represented by a point cloud cluster,
   * meets the height criteria with respect to the provided plane.
   *
   * @param cone_point_cloud Pointer to a Cluster object representing the point cloud of the cone.
   * @param plane The plane against which the cone's height is evaluated.
   * @return true if the cone satisfies the height criteria, false otherwise.
   */
  bool cone_validator(Cluster* cone_point_cloud, Plane& plane) const override;
};

