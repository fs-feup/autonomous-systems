#pragma once

#include <cone_validator/cone_validator.hpp>
#include <utils/cluster.hpp>

/**
 * @brief The ZScoreValidator class is responsible for validating cones based on
 *  the approximation of z-score of when compared with some thresholds.
 * 
 */
class ZScoreValidator : public ConeValidator {
 private:
  double _min_z_score_x_; ///< Min z-score on x-axis */
  double _max_z_score_x_; ///< Max z-score on x-axis */
  double _min_z_score_y_; ///< Min z-score on y-axis */
  double _max_z_score_y_; ///< Max z-score on y-axis */

 public:

  /**
   * @brief Construct a new ZScoreValidator object with the specific thresholds
   * 
   * @param min_z_score_x Min z-score on x-axis
   * @param max_z_score_x Max z-score on x-axis
   * @param min_z_score_y Min z-score on y-axis
   * @param max_z_score_y Max z-score on y-axis
   */
  ZScoreValidator(double min_z_score_x, double max_z_score_x, double min_z_score_y, double max_z_score_y);

  /**
   * @brief Validates the cluster based on the z-score distribution
   * 
   * @param cluster Cluster to be validated
   * @return true if the cluster is valid, false otherwise
   */
  bool coneValidator(Cluster* cluster, Plane& plane) const override;

  /**
   * @brief Virtual destructor for ZScoreValidator.
   */
  virtual ~ZScoreValidator() = default;
};

