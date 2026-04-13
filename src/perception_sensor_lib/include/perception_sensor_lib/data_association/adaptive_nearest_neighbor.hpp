#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "common_lib/maths/transformations.hpp"
#include "perception_sensor_lib/data_association/base_data_association.hpp"

/**
 * @brief Data association implementation that first finds the transformation and then associates with the nearest neighbor
 *
 */
class AdaptiveNearestNeighbor : public DataAssociationModel {
public:
  /**
   * @brief Construct an adaptive nearest neighbor data association model.
   *
   * @param params Parameters that control association gates and adaptive seed selection.
   */
  AdaptiveNearestNeighbor(const DataAssociationParameters& params);

  ~AdaptiveNearestNeighbor() = default;

  /**
   * @brief Associate observations to landmarks using an adaptive seed-based alignment step.
   *
   * The method first computes a baseline nearest-neighbor association, then evaluates
   * candidate rigid transforms built from promising observation-landmark seed pairs,
   * and keeps the association result with the best score.
   *
   * @param landmarks Landmark positions in format [x1, y1, x2, y2, ...].
   * @param observations Observation positions in format [x1, y1, x2, y2, ...].
   * @param covariance Observation/state covariance used by the nearest-neighbor backend.
   * @param observation_confidences Confidence value for each observation.
   * @param pose Current vehicle pose used to gate candidate seed observations.
   * @return Eigen::VectorXi of associated landmark indices per observation, or -1 if unassigned.
   */
  Eigen::VectorXi associate(const Eigen::VectorXd& landmarks, const Eigen::VectorXd& observations,
                            const Eigen::MatrixXd& covariance,
                            const Eigen::VectorXd& observation_confidences,
                            const Eigen::Vector3d& pose) const override;

private:
  /**
   * @brief Find the closest landmark candidate for each observation that can be used as a seed.
   *
   * @param landmarks Landmark positions in format [x1, y1, x2, y2, ...].
   * @param observations Observation positions in format [x1, y1, x2, y2, ...].
   * @param observation_confidences Confidence value for each observation.
   * @param pose Current vehicle pose used to discard distant observations.
   * @param nearest_landmarks Output vector with the nearest landmark index per observation.
   * @param nearest_distances Output vector with the nearest distance per observation.
   */
  void get_tentative_nearest_neighbors(const Eigen::VectorXd& landmarks,
                                       const Eigen::VectorXd& observations,
                                       const Eigen::VectorXd& observation_confidences,
                                       const Eigen::Vector3d& pose,
                                       Eigen::VectorXi& nearest_landmarks,
                                       Eigen::VectorXd& nearest_distances) const;

  /**
   * @brief Select the best observation indices to use as transform seeds.
   *
   * @param nearest_landmarks Tentative nearest landmark index per observation.
   * @param nearest_distances Tentative nearest landmark distance per observation.
   * @return Eigen::VectorXi Indices of the selected seed observations.
   */
  Eigen::VectorXi get_seed_observation_indices(const Eigen::VectorXi& nearest_landmarks,
                                               const Eigen::VectorXd& nearest_distances) const;

  /**
   * @brief Estimate a 2D rigid transform from two observation-landmark correspondences.
   *
   * @param landmarks Landmark positions in format [x1, y1, x2, y2, ...].
   * @param observations Observation positions in format [x1, y1, x2, y2, ...].
   * @param first_observation_idx Index of the first seed observation.
   * @param first_landmark_idx Flat index of the first seed landmark in the landmarks vector.
   * @param second_observation_idx Index of the second seed observation.
   * @param second_landmark_idx Flat index of the second seed landmark in the landmarks vector.
   * @param transform Output transform as [tx, ty, theta].
   * @return true if a valid transform was estimated, false otherwise.
   */
  bool estimate_transform_from_pair(const Eigen::VectorXd& landmarks,
                                    const Eigen::VectorXd& observations,
                                    int first_observation_idx, int first_landmark_idx,
                                    int second_observation_idx, int second_landmark_idx,
                                    Eigen::Vector3d& transform) const;

  /**
   * @brief Score a set of associations by match count and total Euclidean distance.
   *
   * @param landmarks Landmark positions in format [x1, y1, x2, y2, ...].
   * @param observations Observation positions in format [x1, y1, x2, y2, ...].
   * @param associations Associated landmark index per observation.
   * @param num_associations Output number of valid associations.
   * @param total_distance Output accumulated Euclidean distance of all valid matches.
   */
  void score_associations(const Eigen::VectorXd& landmarks, const Eigen::VectorXd& observations,
                          const Eigen::VectorXi& associations, int& num_associations,
                          double& total_distance) const;

  /**
   * @brief Apply a 2D rigid transform to all observations.
   *
   * @param observations Observation positions in format [x1, y1, x2, y2, ...].
   * @param transform Transform to apply as [tx, ty, theta].
   * @return Eigen::VectorXd Transformed observation positions.
   */
  Eigen::VectorXd adjust_observations(const Eigen::VectorXd& observations,
                                      const Eigen::Vector3d& transform) const;
};
