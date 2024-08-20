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

  float max_landmark_distance_; /**< Maximum deviation of the landmark position
                                from the expected position when the landmark is
                                perceived to be 1 meter away */
protected:
  float get_max_landmark_distance() const;

public:
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
 * @brief Simple (Dumb) Maximum Likelihood Method class,
 * attempts to match the observed landmark to the closest landmark in the map.
 * Uses an exponential function to cut off matches whose error is too great for their distance
 */
class MaxLikelihood : public DataAssociationModel {
  // bool valid_match(const float delta, const float distance_to_vehicle) const override;

public:
  static float nis_gate_;  /// normalized innovation squared gate
  static float nd_gate_;   /// normalized distance gate (closest unmatched landmark)

  int associate_n_filter(const std::vector<common_lib::structures::Cone> &perception_map,
                         Eigen::VectorXf &_x_vector_, Eigen::MatrixXf &_p_matrix_,
                         std::vector<int> &matched_ids,
                         std::vector<Eigen::Vector2f> &matched_cone_positions,
                         std::vector<Eigen::Vector2f> &new_features,
                         ObservationModel *observation_model) const override;

  explicit MaxLikelihood(float max_landmark_distance);

  // FRIEND_TEST(DATA_ASSOCIATION_MODEL, VALID_MATCH_FUNC_PERFECT_MATCH);
  // FRIEND_TEST(DATA_ASSOCIATION_MODEL, VALID_MATCH_FUNC_NEAR_MATCH);
  // FRIEND_TEST(DATA_ASSOCIATION_MODEL, VALID_MATCH_FUNC_FAILED_MATCH);
};

const std::map<std::string,
               std::function<std::shared_ptr<DataAssociationModel>(float max_landmark_distance)>,
               std::less<>>
    data_association_model_constructors = {
        {"simple_ml", [](float max_landmark_distance) -> std::shared_ptr<DataAssociationModel> {
           return std::make_shared<MaxLikelihood>(max_landmark_distance);
         }}};