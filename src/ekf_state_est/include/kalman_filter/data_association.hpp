#pragma once

#include <gtest/gtest_prod.h>

#include <Eigen/Dense>
#include <functional>
#include <map>
#include <memory>

#include "common_lib/structures/cone.hpp"
#include "kalman_filter/observation_models.hpp"

/**
 * @brief Data Association Method class,
 * used to match observations to landmarks in the map
 */
class DataAssociationModel {
  /**
   * @brief Check if the observed landmark is a valid match to the expected
   * landmark It depends on external parameters like the distance to the vehicle
   * @param delta The distance between the observed landmark and the expected
   * @param distance_to_vehicle The distance between the vehicle and the
   * observed landmark
   * @return bool True if the observed landmark is a valid match to the expected
   * landmark
   */
  // virtual bool valid_match(const float delta, const float distance_to_vehicle) const = 0;

  /** Maximum distance from the car to a cone for it to be considered in the State Estimation. Cones
   * farther away will be ignored.*/
  float max_landmark_distance_;

protected:
  float get_max_landmark_distance() const;

public:
  /**
   * @brief Associate the observed landmarks to the expected landmarks
   * and update the state vector and the covariance matrix
   * @param perception_map The observed landmarks
   * @param _x_vector_ The state vector
   * @param _p_matrix_ The covariance matrix
   * @param matched_ids Vector in which the ids of the matched landmarks will be placed
   * @param matched_cone_positions Vector in which the positions of the matched landmarks will be
   * placed
   * @param new_features Vector in which the positions of the new landmarks (unmatched landmarks)
   * will be placed
   * @param observation_model The observation model
   * @return int Always 0
   */
  virtual int associate_n_filter(const std::vector<common_lib::structures::Cone> &perception_map,
                                 Eigen::VectorXf &_x_vector_, Eigen::MatrixXf &_p_matrix_,
                                 std::vector<int> &matched_ids,
                                 std::vector<Eigen::Vector2f> &matched_cone_positions,
                                 std::vector<Eigen::Vector2f> &new_features,
                                 ObservationModel *observation_model) const = 0;
  explicit DataAssociationModel(float max_landmark_distance);

  virtual ~DataAssociationModel() = default;
};

/**
 * @brief Maximum Likelihood Method class,
 * used to match observations to landmarks in the map with maximum likelihood method
 * It uses the Mahalanobis distance to determine the best match
 * It also uses a gate to determine if the match is valid
 * The Mahalanobis distance is calculated as the square root of the innovation covariance
 * The gate is a threshold that the Mahalanobis distance must be below to be considered a valid
 * match The gate is defined as the normalized innovation squared (NIS) gate Normalized Distance
 */
class MaxLikelihood : public DataAssociationModel {
  // bool valid_match(const float delta, const float distance_to_vehicle) const override;

public:
  static float association_gate_;   /// normalized innovation squared gate
  static float new_landmark_gate_;  /// normalized distance gate (closest unmatched landmark)

  /**
   * @brief Associate the observed landmarks to the expected landmarks
   * and update the state vector and the covariance matrix
   * @param perception_map The observed landmarks
   * @param _x_vector_ The state vector
   * @param _p_matrix_ The covariance matrix
   * @param matched_ids Vector in which the ids of the matched landmarks will be placed
   * @param matched_cone_positions Vector in which the positions of the matched landmarks will be
   * placed
   * @param new_features Vector in which the positions of the new landmarks (unmatched landmarks)
   * will be placed
   * @param observation_model The observation model
   * @return int Always 0
   */
  int associate_n_filter(const std::vector<common_lib::structures::Cone> &perception_map,
                         Eigen::VectorXf &_x_vector_, Eigen::MatrixXf &_p_matrix_,
                         std::vector<int> &matched_ids,
                         std::vector<Eigen::Vector2f> &matched_cone_positions,
                         std::vector<Eigen::Vector2f> &new_features,
                         ObservationModel *observation_model) const override;

  /**
   * @brief Construct a new Max Likelihood object
   * @param max_landmark_distance Maximum distance from the car to an observed landmark for
   * it to be considered in the state estimation
   */
  explicit MaxLikelihood(float max_landmark_distance);

  // FRIEND_TEST(DATA_ASSOCIATION_MODEL, VALID_MATCH_FUNC_PERFECT_MATCH);
  // FRIEND_TEST(DATA_ASSOCIATION_MODEL, VALID_MATCH_FUNC_NEAR_MATCH);
  // FRIEND_TEST(DATA_ASSOCIATION_MODEL, VALID_MATCH_FUNC_FAILED_MATCH);
};

const std::map<std::string,
               std::function<std::shared_ptr<DataAssociationModel>(float max_landmark_distance)>,
               std::less<>>
    data_association_model_constructors = {
        {"max_likelihood",
         [](float max_landmark_distance) -> std::shared_ptr<DataAssociationModel> {
           return std::make_shared<MaxLikelihood>(max_landmark_distance);
         }}};