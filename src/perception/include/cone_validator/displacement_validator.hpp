#pragma once
#include <cone_validator/cone_validator.hpp>

/**
 * @brief The SizeValidator class is responsible for validating cones based on the minimum distance
 * between the the highest and lowest point cloud points in all axis.
 *
 * This class inherits from the ConeValidator class and overrides the coneValidator method
 * to provide size-based validation logic.
 */
class DisplacementValidator : public ConeValidator {
private:
  double _min_distance_x_;
  double _min_distance_y_;
  double _min_distance_z_;

public:
  /**
   * @brief Constructs a new DisplacementValidator object with the specified distance threshold for
   * all axis.
   *
   * @param min_distance_x Minimum distance between the highest an lowest point on the x axis.
   * @param min_distance_y Minimum distance between the highest an lowest point on the y axis.
   * @param min_distance_z Minimum distance between the highest an lowest point on the z axis.
   */
  explicit DisplacementValidator(double min_distance_x, double min_distance_y,
                                 double min_distance_z);

  /**
   * @brief Validates a cone based on the maximum distance between its points in all axis.
   *
   * This method overrides the coneValidator method of the base class ConeValidator.
   * It validates whether the given cone, represented by a point cloud cluster,
   * meets the minimum distance criteria in all axis.
   *
   * @param cone_point_cloud Pointer to a Cluster object representing the point cloud of the cone.
   * @return vector containing:
   * Index 0 -> ratio between the x axis displacement and the minimum distance for that axis. |
   * Index 1 -> ratio between the y axis displacement and the minimum distance for that axis. |
   * Index 2 -> ratio between the z axis displacement and the minimum distance for that axis.
   */
  std::vector<double> coneValidator(Cluster* cone_point_cloud, Plane& plane) const override;

  /**
   * @brief Virtual destructor for DisplacementValidator.
   */
  virtual ~DisplacementValidator() = default;
};