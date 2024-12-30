#pragma once

#include <cone_validator/cone_validator.hpp>
#include <utils/cluster.hpp>

/**
 * @class CylinderValidator
 * @brief Validates clusters using a cylinder approximation.
 *
 * The CylinderValidator class inherits from the ConeValidator class and implements
 * cluster validation using a cylinder approximation. It provides methods to set
 * and retrieve cylinder dimensions and to perform validation of clusters.
 */
class CylinderValidator : public ConeValidator {
private:
  double small_width;      /**< Width of the cylinder for a small cone. */
  double small_height;     /**< Height of the cylinder for a small cone. */
  double large_width;      /**< Width of the cylinder for a large cone. */
  double large_height;     /**< Height of the cylinder for a large cone. */
  double out_distance_cap; /**< Minimum out_distance value for it to be 0*/

public:
  /**
   * @brief Constructs a new CylinderValidator object with specified width and height.
   * @param small_width The width of the cylinder for a small cone.
   * @param small_height The height of the cylinder for a small cone.
   * @param large_width The width of the cylinder for a large cone.
   * @param large_height The height of the cylinder for a large cone.
   */
  CylinderValidator(double small_width, double small_height, double large_width,
                    double large_height, double out_distance_cap);

  /**
   * @brief Gets the radius of the cylinder for small cones.
   * @return The radius of the cylinder for small cones.
   */
  double small_getRadius() const;

  /**
   * @brief Gets the radius of the cylinder for large cones.
   * @return The radius of the cylinder for large cones.
   */
  double large_getRadius() const;

  /**
   * @brief Validates a cluster using cylinder approximation.
   * @param cone_point_cloud Pointer to the cluster to be validated.
   * @return True if the cluster is valid, false otherwise.
   */
  std::vector<double> coneValidator(Cluster* cone_point_cloud, Plane& plane) const override;
};
