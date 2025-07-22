#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "common_lib/maths/transformations.hpp"
#include "perception_sensor_lib/data_association/base_data_association.hpp"

/**
 * @brief Data association implementation that uses the Malhanobis Distance only
 * as criterion to make observation matches.
 *
 */
class JCBB : public DataAssociationModel {
  /**
   * @brief Recursive branch and bound search to find the best hypothesis.
   *
   * @param current_obs_idx Index of the observation being processed.
   * @param num_observations Total number of observations.
   * @param num_landmarks Total number of landmarks.
   * @param distances Precomputed Euclidean distances between observations and landmarks.
   * @param current_hypothesis Current hypothesis (partial).
   * @param best_hypothesis Best hypothesis found so far (output).
   * @param best_score Best score found so far (output).
   * @param current_score Score of the current partial hypothesis.
   */
  void search_branch_and_bound(int current_obs_idx, int num_observations, int num_landmarks,
                               const Eigen::MatrixXd& distances,
                               Eigen::VectorXi& current_hypothesis,
                               Eigen::VectorXi& best_hypothesis, double& best_score,
                               double current_score) const;
  /**
   * @brief Check if a landmark is already assigned in the current hypothesis.
   *
   * @param hypothesis Current hypothesis vector.
   * @param landmark_idx Index of the landmark to check.
   * @return true if landmark is already assigned, false otherwise.
   */
  bool is_landmark_already_assigned(const Eigen::VectorXi& hypothesis, int landmark_idx) const;

public:
  JCBB(const DataAssociationParameters& params) : DataAssociationModel(params) {}

  ~JCBB() = default;
  /**
   * @brief Perform data association using the Joint Compatibility Branch & Bound algorithm.
   *
   * This function finds the most consistent assignment of 2D observations
   * to known 2D landmarks by maximizing the number of mutually compatible pairs.
   * Uses Euclidean distance as the compatibility measure and a branch & bound search
   * to efficiently explore the space of hypotheses.
   *
   * @param landmark_positions Landmark positions in format [x1, y1, x2, y2, ...].
   * @param observation_positions Observation positions in format [x1, y1, x2, y2, ...].
   * @param covariance Covariance matrix of state estimate (unused in this version).
   * @param observation_confidences Confidences for each observation (unused in this version).
   * @return Eigen::VectorXi of size num_observations:
   *         - If element i >= 0 → index of associated landmark.
   *         - If element i == -1 → observation is unassigned (new/clutter).
   */
  Eigen::VectorXi associate(const Eigen::VectorXd& landmarks, const Eigen::VectorXd& observations,
                            const Eigen::MatrixXd& covariance,
                            const Eigen::VectorXd& observation_confidences) const override;
};