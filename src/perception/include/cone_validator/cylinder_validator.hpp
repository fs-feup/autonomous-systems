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
  double width;  /**< Width of the cylinder. */
  double height; /**< Height of the cylinder. */

 public:
  /**
   * @brief Constructs a new CylinderValidator object with specified width and height.
   * @param width The width of the cylinder.
   * @param height The height of the cylinder.
   */
  CylinderValidator(double width, double height);

  /**
   * @brief Gets the radius of the cylinder.
   * @return The radius of the cylinder.
   */
  double getRadius() const;

  /**
   * @brief Validates a cluster using cylinder approximation.
   * @param cone_point_cloud Pointer to the cluster to be validated.
   * @return True if the cluster is valid, false otherwise.
   */
  bool coneValidator(Cluster* cone_point_cloud, Plane& plane) const override;
};

