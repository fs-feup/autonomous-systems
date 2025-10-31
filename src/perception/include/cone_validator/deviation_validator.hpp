#pragma once

#include <cone_validator/cone_validator.hpp>

/**
 * @class DeviationValidator
 * @brief Validates clusters using the standard deviation on xOy plane and on the z axis.
 *
 * The DeviationValidator class inherits from the ConeValidator class and implements
 * cluster validation using the standard deviation.
 */
class DeviationValidator : public ConeValidator {
private:
  double _min_xoy_; /**< Minimum xOy plane deviation. */
  double _max_xoy_; /**< Maximum xOy plane deviation. */
  double _min_z_;   /**< Minimum z axis deviation. */
  double _max_z_;   /**< Maximum z axis deviation. */

public:
  /**
   * @brief Constructs a new DeviationValidator object with specified intervals on the deviation.
   * @param min_xoy Minimum xOy plane deviation.
   * @param max_xoy Maximum xOy plane deviation.
   * @param min_z Minimum z axis deviation.
   * @param max_z Maximum z axis deviation.
   */
  DeviationValidator(double min_xoy, double max_xoy, double min_z, double max_z);

  /**
   * @brief Validates a cluster using standard deviation.
   * @param cone_point_cloud Pointer to the cluster to be validated.
   * @return vector containing:
   * Index 0 -> if not in xoy interval, ratio between the cluster std deviation and the closest
   * limit on the xoy plane|
   * Index 1 -> if not z in interval, ratio between cluster std deviation and the closest limit on
   * the z axis.
   */
  std::vector<double> coneValidator(Cluster* cone_point_cloud, Plane& plane) const override;

  /**
   * @brief Virtual destructor for DeviationValidator.
   */
  virtual ~DeviationValidator() = default;
};
