#include "perception_sensor_lib/data_association/jcbb.hpp"

Eigen::VectorXi JCBB::associate(const Eigen::VectorXd& landmarks,
                                const Eigen::VectorXd& observations,
                                const Eigen::MatrixXd& covariance,
                                const Eigen::VectorXd& observation_confidences) const {
  const int num_observations = observations.size() / 2;
  const int num_landmarks = landmarks.size() / 2;

  Eigen::VectorXi best_hypothesis =
      Eigen::VectorXi::Constant(num_observations, -1);  // -1 means new observation
  Eigen::VectorXi current_hypothesis = Eigen::VectorXi::Constant(num_observations, -1);

  double best_score = -1.0;

  // Precompute all Euclidean distances between observations and landmarks
  Eigen::MatrixXd distances(num_observations, num_landmarks);
  for (int obs_idx = 0; obs_idx < num_observations; ++obs_idx) {
    Eigen::Vector2d observation(observations(2 * obs_idx), observations(2 * obs_idx + 1));
    for (int lm_idx = 0; lm_idx < num_landmarks; ++lm_idx) {
      Eigen::Vector2d landmark(landmarks(2 * lm_idx), landmarks(2 * lm_idx + 1));
      distances(obs_idx, lm_idx) = (observation - landmark).norm();
    }
  }

  // Start the recursive branch & bound search
  search_branch_and_bound(0, num_observations, num_landmarks, distances, current_hypothesis,
                          best_hypothesis, best_score, 0.0);

  return best_hypothesis;
}

void JCBB::search_branch_and_bound(int current_obs_idx, int num_observations, int num_landmarks,
                                   const Eigen::MatrixXd& distances,
                                   Eigen::VectorXi& current_hypothesis,
                                   Eigen::VectorXi& best_hypothesis, double& best_score,
                                   double current_score) const {
  if (current_obs_idx >= num_observations) {
    if (current_score > best_score) {
      best_hypothesis = current_hypothesis;
      best_score = current_score;
    }
    return;
  }

  // Try pairing current observation with each landmark
  for (int landmark_idx = 0; landmark_idx < num_landmarks; ++landmark_idx) {
    if (distances(current_obs_idx, landmark_idx) <= this->_params_.association_gate &&
        !is_landmark_already_assigned(current_hypothesis, landmark_idx)) {
      current_hypothesis(current_obs_idx) = 2 * landmark_idx;

      double updated_score = current_score + (this->_params_.association_gate -
                                              distances(current_obs_idx, landmark_idx));

      // Upper bound estimate: assume all remaining observations are perfect
      double upper_bound = updated_score + (num_observations - current_obs_idx - 1) *
                                               this->_params_.association_gate;

      if (upper_bound >= best_score) {
        search_branch_and_bound(current_obs_idx + 1, num_observations, num_landmarks, distances,
                                current_hypothesis, best_hypothesis, best_score, updated_score);
      }

      // backtrack
      current_hypothesis(current_obs_idx) = -1;
    }
  }

  // Or leave the current observation unpaired (new or clutter)
  current_hypothesis(current_obs_idx) = -1;

  double upper_bound =
      current_score + (num_observations - current_obs_idx - 1) * this->_params_.association_gate;

  if (upper_bound >= best_score) {
    search_branch_and_bound(current_obs_idx + 1, num_observations, num_landmarks, distances,
                            current_hypothesis, best_hypothesis, best_score, current_score);
  }

  // backtrack (redundant, but clear)
  current_hypothesis(current_obs_idx) = -1;
}

bool JCBB::is_landmark_already_assigned(const Eigen::VectorXi& hypothesis, int landmark_idx) const {
  for (int i = 0; i < hypothesis.size(); ++i) {
    if (hypothesis(i) == landmark_idx) return true;
  }
  return false;
}
