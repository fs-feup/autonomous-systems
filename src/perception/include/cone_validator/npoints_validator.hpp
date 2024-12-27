#pragma once

#include <cone_validator/cone_validator.hpp>
#include <utils/cluster.hpp>

/**
 * @brief The NPointsValidator class is responsible for validating cones based on the number of
 * points they contain.
 *
 * This class inherits from the ConeValidator class and overrides the coneValidator method
 * to provide point quantity validation logic.
 */
class NPointsValidator : public ConeValidator {
private:
  long unsigned int _min_n_points_;

public:
  /**
   * @brief Constructs a new HeightValidator object with the specified height threshold.
   *
   * @param min_n_points Min number of points for cone validation.
   */
  explicit NPointsValidator(long unsigned int min_n_points);

  /**
   * @brief Validates a cone based on its number of points.
   *
   * This method overrides the coneValidator method of the base class ConeValidator.
   * It validates whether the given cone, represented by a point cloud cluster,
   * meets the minimum number of points criteria.
   *
   * @param cone_point_cloud Pointer to a Cluster object representing the point cloud of the cone.
   * @return true if the cone satisfies the number of points criteria, false otherwise.
   */
  std::vector<double> coneValidator(Cluster* cone_point_cloud, Plane& plane) const override;

  /**
   * @brief Virtual destructor for NpointsValidator.
   */
  virtual ~NPointsValidator() = default;
};