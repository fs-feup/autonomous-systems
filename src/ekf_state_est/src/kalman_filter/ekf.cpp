#define EIGEN_USE_THREADS

#include "kalman_filter/ekf.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "common_lib/maths/matrixes.hpp"

/*---------------------- Constructors --------------------*/

ExtendedKalmanFilter::ExtendedKalmanFilter(
    std::shared_ptr<MotionModel> motion_model, std::shared_ptr<ObservationModel> observation_model,
    std::shared_ptr<DataAssociationModel> data_association_model)
    : _motion_model_(motion_model),
      _observation_model_(observation_model),
      _data_association_model_(data_association_model) {
  Eigen::MatrixXf dense_matrix = Eigen::MatrixXf::Identity(6, 6) * 100.0f;
  _p_matrix_ = dense_matrix.sparseView();
  _x_vector_ = Eigen::VectorXf::Constant(6, 0.05);
  std::ofstream file("matrices.txt", std::ios::app);
  file << "\n";
}
/*-----------------------Algorithms-----------------------*/

void ExtendedKalmanFilter::prediction_step(const MotionUpdate &motion_update) {
  rclcpp::Time temp_this_last_update = this->_last_update_;
  this->_last_update_ = motion_update.last_update;
  if (this->_first_prediction_) {  // To set the last update correctly
    this->_first_prediction_ = false;
    return;
  }
  // print the time values and the difference
  RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "TIMES: %f / %f / %f",
               this->_last_update_.seconds(), temp_this_last_update.seconds(),
               (this->_last_update_ - temp_this_last_update).seconds());

  double delta = (this->_last_update_ - temp_this_last_update).seconds();
  Eigen::VectorXf temp_x_vector = this->_x_vector_;
  std::ofstream file("matrices.txt", std::ios::app);
  file << "PREDICTION STEP\n\n";
  file << "Previous State\n" << this->_x_vector_ << "\n";
  this->_x_vector_ =
      this->_motion_model_->predict_expected_state(temp_x_vector, motion_update, delta);
  file << "Predicted State\n" << this->_x_vector_ << "\n";
  Eigen::MatrixXf g_matrix =
      this->_motion_model_->get_motion_to_state_matrix(temp_x_vector, motion_update, delta);
  Eigen::MatrixXf r_matrix = this->_motion_model_->get_process_noise_covariance_matrix(
      static_cast<unsigned int>(this->_x_vector_.size()));  // Process Noise Matrix
  // this->_p_matrix_ = g_matrix * this->_p_matrix_ * g_matrix.transpose() + r_matrix;
  Eigen::MatrixXf p_matrix_dense =
      g_matrix * Eigen::MatrixXf(this->_p_matrix_) * g_matrix.transpose() + r_matrix;
  this->_p_matrix_ = p_matrix_dense.sparseView();
}
void ExtendedKalmanFilter::wss_correction_step(const MotionUpdate &motion_correction_data) {
  // // Matrix Calculation
  // // print current state first 5 elements
  // RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "WSS STATE B4 CORRECT %f %f %f %f %f
  // \n\n\n\n",
  //              this->_x_vector_(0), this->_x_vector_(1), this->_x_vector_(2),
  //              this->_x_vector_(3), this->_x_vector_(4));

  // // print the rest of the state vector
  // for (int i = 6; i < this->_x_vector_.size(); i += 2) {
  //   RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "CONE %d B4: X:%f Y:%f", i,
  //                this->_x_vector_(i), this->_x_vector_(i + 1));
  // }

  // Eigen::MatrixXf low_jacobian = Eigen::MatrixXf::Zero(2, 6);
  // low_jacobian(0, 3) = 1;
  // low_jacobian(1, 4) = 1;

  // Eigen::MatrixXf transformation_matrix = Eigen::MatrixXf::Zero(6, this->_x_vector_.size());
  // transformation_matrix(0, 0) = 1;
  // transformation_matrix(1, 1) = 1;
  // transformation_matrix(2, 2) = 1;
  // transformation_matrix(3, 3) = 1;
  // transformation_matrix(4, 4) = 1;
  // transformation_matrix(5, 5) = 1;
  // Eigen::MatrixXf h_matrix = low_jacobian * transformation_matrix;

  // Eigen::MatrixXf r_matrix = Eigen::MatrixXf::Zero(2, 2);

  // Eigen::MatrixXf kalman_gain_matrix = this->get_kalman_gain(h_matrix, this->_p_matrix_,
  // r_matrix);

  // // Correction
  // Eigen::Vector2f u_hat(this->_x_vector_(3), this->_x_vector_(4));
  // // Convert translational velocity to Cartesian using the car's orientation
  // float theta = this->_x_vector_[2];
  // float Vx_observed = motion_correction_data.translational_velocity * cos(theta);
  // float Vy_observed = motion_correction_data.translational_velocity * sin(theta);
  // RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "VX VY %f %f", Vx_observed, Vy_observed);

  // Eigen::Vector2f u(Vx_observed, Vy_observed);
  // std::ofstream file("matrices.txt", std::ios::app);
  // file << "WSS CORRECTION STEP\n\n";
  // file << "_x_vector_ before correction WSS:\n" << this->_x_vector_ << "\n";
  // RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "U - UHAT %f %f", (u - u_hat)(0),
  //              (u - u_hat)(1));
  // this->_x_vector_ = this->_x_vector_ + kalman_gain_matrix * (u - u_hat);

  // file << "_x_vector_ after correction WSS:\n" << this->_x_vector_ << "\n";

  // RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "WSS STATE AFTER CORRECT %f %f %f %f %f",
  //              this->_x_vector_(0), this->_x_vector_(1), this->_x_vector_(2),
  //              this->_x_vector_(3), this->_x_vector_(4));

  // for (int i = 6; i < this->_x_vector_.size(); i += 2) {
  //   RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "CONE %d AFTER: X:%f Y:%f", i,
  //                this->_x_vector_(i), this->_x_vector_(i + 1));
  // }
  // this->_p_matrix_ = (Eigen::MatrixXf::Identity(this->_p_matrix_.rows(), this->_p_matrix_.cols())
  // -
  //                     kalman_gain_matrix * h_matrix)
  //                        .sparseView() *
  //                    this->_p_matrix_;

  // file << "P WSS posterior Matrix:\n" << this->_p_matrix_ << "\n";
  // file.close();

  rclcpp::Time temp_this_last_update = this->_last_update_;
  this->_last_update_ = motion_correction_data.last_update;
  if (this->_first_prediction_) {  // To set the last update correctly
    this->_first_prediction_ = false;
    return;
  }
  // print the time values and the difference
  RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "TIMES: %f / %f / %f",
               this->_last_update_.seconds(), temp_this_last_update.seconds(),
               (this->_last_update_ - temp_this_last_update).seconds());

  double delta = (this->_last_update_ - temp_this_last_update).seconds();
  Eigen::VectorXf temp_x_vector = this->_x_vector_;
  std::ofstream file("matrices.txt", std::ios::app);
  file << "WSS PREDICTION STEP\n\n";
  file << "WSS Previous State\n" << this->_x_vector_ << "\n";
  NormalVelocityModel motion_model =
      NormalVelocityModel(this->_motion_model_->get_process_noise_covariance_matrix(6));
  this->_x_vector_ =
      motion_model.predict_expected_state(temp_x_vector, motion_correction_data, delta);
  file << "Predicted State\n" << this->_x_vector_ << "\n";
  Eigen::MatrixXf g_matrix = this->_motion_model_->get_motion_to_state_matrix(
      temp_x_vector, motion_correction_data, delta);
  Eigen::MatrixXf r_matrix = this->_motion_model_->get_process_noise_covariance_matrix(
      static_cast<unsigned int>(this->_x_vector_.size()));  // Process Noise Matrix
  // this->_p_matrix_ = g_matrix * this->_p_matrix_ * g_matrix.transpose() + r_matrix;
  Eigen::MatrixXf p_matrix_dense =
      g_matrix * Eigen::MatrixXf(this->_p_matrix_) * g_matrix.transpose() + r_matrix;
  this->_p_matrix_ = p_matrix_dense.sparseView();
}

// std::ofstream file("matrices.txt", std::ios::app);
// file << "CORRECTION STEP\n\n";
// std::vector<ObservationData> valid_observations;
// std::vector<int> landmark_indices;
// for (const common_lib::structures::Cone &cone : perception_map) {
//   ObservationData observation_data =
//       ObservationData(cone.position.x, cone.position.y, cone.color);

//   file << "Observation Data\n"
//        << observation_data.position.x << "//" << observation_data.position.y << "\n";

//   // Data Association
//   Eigen::Vector2f landmark_absolute =
//       this->_observation_model_->inverse_observation_model(this->_x_vector_, observation_data);
//   int landmark_index = _data_association_model_->match_cone(landmark_absolute,
//   this->_x_vector_); if (landmark_index >= 0) {
//     file << "MATCH\n\n";
//     valid_observations.push_back(observation_data);
//     landmark_indices.push_back(landmark_index);
//   } else if (landmark_index == -1) {  // Too far away landmark
//     continue;
//   } else if (landmark_index == -2) {  // Did not match any landmark
//     this->_x_vector_.conservativeResizeLike(Eigen::VectorXf::Zero(this->_x_vector_.size() +
//     2)); this->_x_vector_(this->_x_vector_.size() - 2) = landmark_absolute(0);
//     this->_x_vector_(this->_x_vector_.size() - 1) = landmark_absolute(1);
//     Eigen::SparseMatrix<float> new_p_matrix(this->_p_matrix_.rows() + 2,
//                                             this->_p_matrix_.cols() + 2);
//     common_lib::maths::copy_eigen_sparse_matrix(this->_p_matrix_, new_p_matrix);
//     this->_p_matrix_.swap(new_p_matrix);
//     landmark_index = static_cast<int>(this->_x_vector_.size() - 2);
//   }
// }

// if (valid_observations.empty()) {
//   return;
// }

// int num_landmarks = static_cast<int>(valid_observations.size());
// Eigen::MatrixXf Q = this->_observation_model_->get_observation_noise_covariance_matrix();
// Eigen::MatrixXf H = Eigen::MatrixXf::Zero(num_landmarks * 2, this->_x_vector_.size());
// Eigen::VectorXf Z = Eigen::VectorXf::Zero(num_landmarks * 2);      // Actual observations
// Eigen::VectorXf z_hat = Eigen::VectorXf::Zero(num_landmarks * 2);  // Predicted observations

// Eigen::MatrixXf K_sum = Eigen::MatrixXf::Zero(_x_vector_.size(), 2 * num_landmarks);

// for (int i = 0; i < num_landmarks; ++i) {
//   int landmark_index = landmark_indices[i];
//   Eigen::Vector2f predicted_landmark_position =
//       this->_observation_model_->observation_model(this->_x_vector_, landmark_index);

//   Eigen::MatrixXf H_i = this->_observation_model_->get_state_to_observation_matrix(
//       this->_x_vector_, landmark_index, static_cast<unsigned int>(this->_x_vector_.size()));

//   Z(2 * i) = valid_observations[i].position.x;
//   Z(2 * i + 1) = valid_observations[i].position.y;

//   z_hat(2 * i) = predicted_landmark_position(0);
//   z_hat(2 * i + 1) = predicted_landmark_position(1);

//   H.block(2 * i, 0, 2, H_i.cols()) = H_i;

//   Eigen::MatrixXf S_i = H_i * _p_matrix_ * H_i.transpose() + Q;
//   Eigen::MatrixXf K_i = _p_matrix_ * H_i.transpose() * S_i.inverse();

//   K_sum.block(0, 2 * i, _x_vector_.size(), 2) = K_i;
// }

// file << "Previous State\n" << this->_x_vector_ << "\n";
// this->_x_vector_ += K_sum * (Z - z_hat);

// this->_p_matrix_ =
//     ((Eigen::MatrixXf::Identity(_p_matrix_.rows(), _p_matrix_.cols()) - K_sum * H) *
//     _p_matrix_)
//         .sparseView();

// file << "CORRECTED  STATE\n" << _x_vector_ << "\n";
// file << "P posterior Matrix:\n" << this->_p_matrix_.toDense() << "\n";
// file.close();

void ExtendedKalmanFilter::correction_step(
    const std::vector<common_lib::structures::Cone> &perception_map) {
  std::ofstream file("matrices.txt", std::ios::app);
  file << "CORRECTION STEP\n\n";
  file << "Previous State\n" << this->_x_vector_ << "\n";
  file.flush();
  for (const common_lib::structures::Cone &cone : perception_map) {
    ObservationData observation_data =
        ObservationData(cone.position.x, cone.position.y, cone.color);

    file << "Observation Data\n"
         << observation_data.position.x << "//" << observation_data.position.y << "\n";

    // Data Association
    Eigen::Vector2f landmark_absolute =
        this->_observation_model_->inverse_observation_model(this->_x_vector_, observation_data);
    int landmark_index = _data_association_model_->match_cone(landmark_absolute, this->_x_vector_);
    if (landmark_index == -1) {  // Too far away or too unseen landmark
      continue;
    } else if (landmark_index == -2) {  // Did not match any landmark
      file << "NEW LANDMARK\n\n";
      this->_x_vector_.conservativeResizeLike(Eigen::VectorXf::Zero(this->_x_vector_.size() + 2));
      this->_x_vector_(this->_x_vector_.size() - 2) = landmark_absolute(0);
      this->_x_vector_(this->_x_vector_.size() - 1) = landmark_absolute(1);
      Eigen::SparseMatrix<float> new_p_matrix(this->_p_matrix_.rows() + 2,
                                              this->_p_matrix_.cols() + 2);
      common_lib::maths::copy_eigen_sparse_matrix(this->_p_matrix_, new_p_matrix);
      this->_p_matrix_.swap(new_p_matrix);
      landmark_index = static_cast<int>(this->_x_vector_.size() - 2);
    }

    // Matrix Calculation
    file << "H Matrix:" << "\n";
    file.flush();
    Eigen::MatrixXf h_matrix = this->_observation_model_->get_state_to_observation_matrix(
        this->_x_vector_, landmark_index, static_cast<unsigned int>(this->_x_vector_.size()));
    file << "Q Matrix:" << "\n";
    file.flush();
    Eigen::MatrixXf q_matrix =
        this->_observation_model_->get_observation_noise_covariance_matrix();  // Observation Noise
                                                                               // Matrix
    file << "K Matrix:" << "\n";
    file.flush();
    Eigen::MatrixXf kalman_gain_matrix =
        this->get_kalman_gain(h_matrix, this->_p_matrix_, q_matrix);

    // Correction
    Eigen::Vector2f z_hat = this->_observation_model_->observation_model(
        this->_x_vector_, landmark_index);  // expected observation, previously
                                            // calculated landmark is used
    Eigen::Vector2f z =
        Eigen::Vector2f(observation_data.position.x,
                        observation_data.position.y);  // position of the landmark  observed
    this->_x_vector_ = this->_x_vector_ + kalman_gain_matrix * (z - z_hat);
    this->_p_matrix_ =
        (Eigen::MatrixXf::Identity(this->_p_matrix_.rows(), this->_p_matrix_.cols()) -
         kalman_gain_matrix * h_matrix)
            .sparseView() *
        this->_p_matrix_;
  }

  file << "CORRECTED  STATE\n" << _x_vector_.head(6) << "\n";
  // file << "P posterior Matrix:\n" << this->_p_matrix_.toDense() << "\n";
  file.close();
}

Eigen::MatrixXf ExtendedKalmanFilter::get_kalman_gain(const Eigen::MatrixXf &h_matrix,
                                                      const Eigen::MatrixXf &p_matrix,
                                                      const Eigen::MatrixXf &q_matrix) const {
  // std::ofstream file("matrices.txt", std::ios::app);
  // file << "H Matrix:\n" << h_matrix << "\n";
  // file << "P Matrix:\n" << p_matrix << "\n";
  // file << "Q Matrix:\n" << q_matrix << "\n";

  Eigen::MatrixXf s_matrix = h_matrix * p_matrix * h_matrix.transpose() + q_matrix;
  // file << "S Matrix:\n" << s_matrix << "\n";

  if ((s_matrix.array() == 0).all()) {
    // file << "S Matrix is a zero matrix. Returning zero Kalman Gain Matrix.\n";
    return Eigen::MatrixXf::Zero(p_matrix.rows(), h_matrix.rows());
  }
  Eigen::MatrixXf kalman_gain_matrix = p_matrix * h_matrix.transpose() * s_matrix.inverse();
  // file << "Kalman Gain Matrix:\n" << kalman_gain_matrix << "\n";

  // file.close();
  return kalman_gain_matrix;
}

void ExtendedKalmanFilter::update(
    std::shared_ptr<common_lib::structures::VehicleState> vehicle_state,
    std::shared_ptr<std::vector<common_lib::structures::Cone>> track_map) {
  vehicle_state->pose.position.x = this->_x_vector_(0);
  vehicle_state->pose.position.y = this->_x_vector_(1);
  vehicle_state->pose.orientation = this->_x_vector_(2);
  vehicle_state->velocity_x = this->_x_vector_(3);
  vehicle_state->velocity_y = this->_x_vector_(4);
  vehicle_state->rotational_velocity = this->_x_vector_(5);
  track_map->clear();
  for (int i = 6; i < this->_x_vector_.size() - 1; i += 2) {
    track_map->push_back(
        common_lib::structures::Cone(_x_vector_(i), _x_vector_(i + 1), "UNKNOWN", 1.0));
  }
}
