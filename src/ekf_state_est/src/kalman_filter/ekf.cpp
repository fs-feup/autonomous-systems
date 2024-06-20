#define EIGEN_USE_THREADS

#include "kalman_filter/ekf.hpp"

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "common_lib/maths/matrixes.hpp"

/*---------------------- Constructors --------------------*/

ExtendedKalmanFilter::ExtendedKalmanFilter(
    std::shared_ptr<ObservationModel> observation_model,
    std::shared_ptr<DataAssociationModel> data_association_model)
    : _observation_model_(observation_model), _data_association_model_(data_association_model) {
  //_p_matrix_ = Eigen::MatrixXf::Identity(6, 6) * 100.0f;
  _x_vector_ = Eigen::VectorXf::Constant(6, 0.001);
}
/*-----------------------Algorithms-----------------------*/

void ExtendedKalmanFilter::prediction_step(const MotionUpdate &motion_update,
                                           const std::string &sensor_type) {
  std::shared_ptr<MotionModel> motion_model = _motion_models_[sensor_type];
  rclcpp::Time temp_this_last_update = this->_last_update_;
  this->_last_update_ = motion_update.last_update;
  if (this->_first_prediction_) {  // To set the last update correctly
    this->_first_prediction_ = false;
    return;
  }

  double delta = (this->_last_update_ - temp_this_last_update).seconds();
  Eigen::VectorXf temp_x_vector = this->_x_vector_;
  this->_x_vector_ = motion_model->predict_expected_state(temp_x_vector, motion_update, delta);

  Eigen::MatrixXf g_matrix =
      motion_model->get_motion_to_state_matrix(temp_x_vector, motion_update, delta);
  Eigen::MatrixXf r_matrix = motion_model->get_process_noise_covariance_matrix(
      static_cast<unsigned int>(this->_x_vector_.size()));  // Process Noise Matrix
  // this->_p_matrix_ = g_matrix * this->_p_matrix_ * g_matrix.transpose() + r_matrix;
  this->_p_matrix_ = g_matrix * Eigen::MatrixXf(this->_p_matrix_) * g_matrix.transpose() + r_matrix;
}
// void ExtendedKalmanFilter::wss_correction_step(const MotionUpdate &motion_correction_data) {
//   rclcpp::Time temp_this_last_update = this->_last_update_;
//   this->_last_update_ = motion_correction_data.last_update;
//   if (this->_first_prediction_) {  // To set the last update correctly
//     this->_first_prediction_ = false;
//     return;
//   }
//   // print the time values and the difference
//   RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "TIMES: %f / %f / %f",
//                this->_last_update_.seconds(), temp_this_last_update.seconds(),
//                (this->_last_update_ - temp_this_last_update).seconds());

//   double delta = (this->_last_update_ - temp_this_last_update).seconds();
//   Eigen::VectorXf temp_x_vector = this->_x_vector_;
//   NormalVelocityModel motion_model =
//       NormalVelocityModel(this->_motion_model_->get_process_noise_covariance_matrix(
//           static_cast<unsigned int>(this->_x_vector_.size())));
//   this->_x_vector_ =
//       motion_model.predict_expected_state(temp_x_vector, motion_correction_data, delta);
//   Eigen::MatrixXf g_matrix = this->_motion_model_->get_motion_to_state_matrix(
//       temp_x_vector, motion_correction_data, delta);
//   Eigen::MatrixXf r_matrix = this->_motion_model_->get_process_noise_covariance_matrix(
//       static_cast<unsigned int>(this->_x_vector_.size()));  // Process Noise Matrix
//   // this->_p_matrix_ = g_matrix * this->_p_matrix_ * g_matrix.transpose() + r_matrix;
//   Eigen::MatrixXf p_matrix_dense =
//       g_matrix * Eigen::MatrixXf(this->_p_matrix_) * g_matrix.transpose() + r_matrix;
//   this->_p_matrix_ = p_matrix_dense.sparseView();
// }

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
// RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "U - UHAT %f %f", (u - u_hat)(0),
//              (u - u_hat)(1));
// this->_x_vector_ = this->_x_vector_ + kalman_gain_matrix * (u - u_hat);

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

// std::vector<ObservationData> valid_observations;
// std::vector<int> landmark_indices;
// for (const common_lib::structures::Cone &cone : perception_map) {
//   ObservationData observation_data =
//       ObservationData(cone.position.x, cone.position.y, cone.color);

//   // Data Association
//   Eigen::Vector2f landmark_absolute =
//       this->_observation_model_->inverse_observation_model(this->_x_vector_, observation_data);
//   int landmark_index = _data_association_model_->match_cone(landmark_absolute,
//   this->_x_vector_); if (landmark_index >= 0) {
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

// this->_x_vector_ += K_sum * (Z - z_hat);

// this->_p_matrix_ =
//     ((Eigen::MatrixXf::Identity(_p_matrix_.rows(), _p_matrix_.cols()) - K_sum * H) *
//     _p_matrix_)
//         .sparseView();

// void ExtendedKalmanFilter::correction_step(
//     const std::vector<common_lib::structures::Cone> &perception_map) {
//   for (const common_lib::structures::Cone &cone : perception_map) {
//     ObservationData observation_data =
//         ObservationData(cone.position.x, cone.position.y, cone.color);

//     // Data Association
//     Eigen::Vector2f landmark_absolute =
//         this->_observation_model_->inverse_observation_model(this->_x_vector_, observation_data);
//     int landmark_index = _data_association_model_->match_cone(landmark_absolute,
//     this->_x_vector_); Eigen::MatrixXf q_matrix =
//     this->_observation_model_->get_observation_noise_covariance_matrix();
//     // init z_hat to 0,0
//     Eigen::Vector2f z_hat = Eigen::Vector2f::Zero();
//     Eigen::MatrixXf h_matrix = Eigen::MatrixXf::Zero(2, this->_x_vector_.size());
//     if (landmark_index >= 0) {
//       z_hat = this->_observation_model_->observation_model(this->_x_vector_,
//                                                            landmark_index);  // to car frame
//       h_matrix = this->_observation_model_->get_state_to_observation_matrix(
//           this->_x_vector_, landmark_index, static_cast<unsigned int>(this->_x_vector_.size()));
//     }
//     // this->_observation_model_->observation_model(this->_x_vector_, landmark_index);
//     // create a vecotr2f with the observed position
//     Eigen::Vector2f z = Eigen::Vector2f(observation_data.position.x,
//     observation_data.position.y); bool valid = _data_association_model_->validate(z_hat, z,
//     this->_p_matrix_, landmark_index,
//                                                     q_matrix, h_matrix);
//     // output valid
//     if (landmark_index == -1) {  // Too far away or too unseen landmark
//       continue;
//     } else if (!valid) {  // Did not match any landmark
//       this->_x_vector_.conservativeResizeLike(Eigen::VectorXf::Zero(this->_x_vector_.size() +
//       2)); this->_x_vector_(this->_x_vector_.size() - 2) = landmark_absolute(0);
//       this->_x_vector_(this->_x_vector_.size() - 1) = landmark_absolute(1);
//       Eigen::MatrixXf new_p_matrix =
//           Eigen::MatrixXf::Zero(this->_p_matrix_.rows() + 2, this->_p_matrix_.cols() + 2);
//       new_p_matrix.block(0, 0, this->_p_matrix_.rows(), this->_p_matrix_.cols()) =
//       this->_p_matrix_; this->_p_matrix_ = new_p_matrix; landmark_index =
//       static_cast<int>(this->_x_vector_.size() - 2); z_hat =
//       this->_observation_model_->observation_model(this->_x_vector_, landmark_index); h_matrix =
//       this->_observation_model_->get_state_to_observation_matrix(
//           this->_x_vector_, landmark_index, static_cast<unsigned int>(this->_x_vector_.size()));
//     }

//     // Matrix Calculation
//     Eigen::MatrixXf kalman_gain_matrix =
//         this->get_kalman_gain(h_matrix, this->_p_matrix_, q_matrix);

//     this->_x_vector_ = this->_x_vector_ + kalman_gain_matrix * (z - z_hat);
//     this->_p_matrix_ =
//         (Eigen::MatrixXf::Identity(this->_p_matrix_.rows(), this->_p_matrix_.cols()) -
//          kalman_gain_matrix * h_matrix) *
//         this->_p_matrix_;
//   }
// }

void ExtendedKalmanFilter::correction_step(
    const std::vector<common_lib::structures::Cone> &perception_map) {
  // print dbg msg with state vars
  RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "STATE B4 CORRECT %f %f %f %f %f %f",
               this->_x_vector_(0), this->_x_vector_(1), this->_x_vector_(2), this->_x_vector_(3),
               this->_x_vector_(4), this->_x_vector_(5));
  // loop through all the cones in the perception map
  // init vector for matched ids and cone positions
  std::vector<int> matched_ids;
  std::vector<Eigen::Vector2f> matched_cone_positions;
  // init vector for new features with new cone positions
  std::vector<Eigen::Vector2f> new_features;
  for (const common_lib::structures::Cone &cone : perception_map) {
    Eigen::Vector2f landmark_absolute = this->_observation_model_->inverse_observation_model(
        this->_x_vector_, ObservationData(cone.position.x, cone.position.y,
                                          common_lib::competition_logic::Color::BLUE));

    float distance_to_vehicle = (landmark_absolute - this->_x_vector_.segment<2>(0)).norm();
    RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "CONE %f %f", cone.position.x,
                 cone.position.y);
    RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "DISTANCE TO VEHICLE %f",
                 distance_to_vehicle);
    if (distance_to_vehicle > 15) {
      continue;
    }
    int j_best = 0;
    float n_best = std::numeric_limits<int>::max();
    float outer = std::numeric_limits<int>::max();
    for (int j = 6; j < this->_x_vector_.size(); j += 2) {
      // get expect observation from the state vector with the observation model
      Eigen::Vector2f z_hat = this->_observation_model_->observation_model(this->_x_vector_, j);
      // print dbg msg z_hat and cone in the samle line
      // RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "Z_HAT X|%f| Y|%f| CONE X|%f| Y|%f|",
      //              z_hat(0), z_hat(1), cone.position.x, cone.position.y);
      // get the state to observation matrix
      Eigen::MatrixXf h_matrix = this->_observation_model_->get_state_to_observation_matrix(
          this->_x_vector_, j, static_cast<unsigned int>(this->_x_vector_.size()));
      // get the observation noise covariance matrix
      Eigen::MatrixXf q_matrix =
          this->_observation_model_->get_observation_noise_covariance_matrix();
      // calculate innovation cone position - z_hat(predicted cone position)
      Eigen::Vector2f innovation = Eigen::Vector2f(cone.position.x, cone.position.y) - z_hat;
      // dbg
      // RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "INNOVATION X|%f| Y|%f|",
      // innovation(0),
      //              innovation(1));
      // calculate the S matrix
      Eigen::MatrixXf s_matrix = h_matrix * this->_p_matrix_ * h_matrix.transpose() + q_matrix;
      // calculate nis, that means normalized innovation squared
      float nis = innovation.transpose() * s_matrix.inverse() * innovation;
      // dbg
      // RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "NIS %f", nis);
      // calculate nd, that means normalized determinant
      float nd = nis + log(s_matrix.determinant());
      // dbg
      // RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "ND %f\n", nd);
      RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "CURRENT CONE: %f %f", cone.position.x,
                   cone.position.y);
      RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "WITH Z_HAT: %f %f", z_hat(0), z_hat(1));
      RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "IN THE MAP REF: %f %f",
                   this->_x_vector_(j), this->_x_vector_(j + 1));
      RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "INNOVATION %f %f", innovation(0),
                   innovation(1));
      RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "NIS %f", nis);
      RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "ND %f", nd);
      if (nis < 5.991 /* gate1 */ && nd < n_best) {
        // dbg with what matched
        RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "MATCHED CONE: %f %f", cone.position.x,
                     cone.position.y);
        RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "WITH Z_HAT: %f %f", z_hat(0), z_hat(1));
        RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "IN THE MAP REF: %f %f",
                     this->_x_vector_(j), this->_x_vector_(j + 1));
        RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "INNOVATION %f %f", innovation(0),
                     innovation(1));
        RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "NIS %f", nis);
        RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "ND %f", nd);
        n_best = nd;
        j_best = j;
      } else if (nis < outer) {
        outer = nis;  // outer is the closest unmatched distance
      }
    }
    if (j_best != 0) {
      // dbg msg with the match cone position and the best match position
      RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "ADDING TO CORRECT MATCHED CONE %f %f",
                   this->_x_vector_(j_best), this->_x_vector_(j_best + 1));
      matched_ids.push_back(j_best);
      matched_cone_positions.push_back(Eigen::Vector2f(cone.position.x, cone.position.y));
    } else if (outer > 10 /* gate2 */) {
      new_features.push_back(Eigen::Vector2f(cone.position.x, cone.position.y));
    }
  }
  // loop through the matched ids and cone positions and make the corrections
  for (int i = 0; i < matched_ids.size(); i++) {
    Eigen::Vector2f z_hat =
        this->_observation_model_->observation_model(this->_x_vector_, matched_ids[i]);
    RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "CORRECTING WITH Z_HAT: %f %f", z_hat(0),
                 z_hat(1));
    Eigen::MatrixXf h_matrix = this->_observation_model_->get_state_to_observation_matrix(
        this->_x_vector_, matched_ids[i], static_cast<unsigned int>(this->_x_vector_.size()));
    Eigen::MatrixXf q_matrix = this->_observation_model_->get_observation_noise_covariance_matrix();
    Eigen::Vector2f z = matched_cone_positions[i];
    RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "MATCHED Z: %f %f", z(0), z(1));
    RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "THE DIFF: %f %f", (z - z_hat)(0),
                 (z - z_hat)(1));

    Eigen::MatrixXf kalman_gain_matrix =
        this->get_kalman_gain(h_matrix, this->_p_matrix_, q_matrix);
    RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "BEFORE CORRECTION %f %f %f %f %f %f",
                 this->_x_vector_(0), this->_x_vector_(1), this->_x_vector_(2), this->_x_vector_(3),
                 this->_x_vector_(4), this->_x_vector_(5));
    this->_x_vector_ = this->_x_vector_ + kalman_gain_matrix * (z - z_hat);
    RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "AFTER CORRECTION %f %f %f %f %f %f",
                 this->_x_vector_(0), this->_x_vector_(1), this->_x_vector_(2), this->_x_vector_(3),
                 this->_x_vector_(4), this->_x_vector_(5));
    this->_p_matrix_ =
        (Eigen::MatrixXf::Identity(this->_p_matrix_.rows(), this->_p_matrix_.cols()) -
         kalman_gain_matrix * h_matrix)
            .sparseView() *
        this->_p_matrix_;
  }

  // loop through the new features and add them to the state vector
  for (int i = 0; i < new_features.size(); i++) {
    RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "Adding to state vector %f %f",
                 new_features[i](0), new_features[i](1));
    int length = this->_x_vector_.size();
    this->_x_vector_.conservativeResizeLike(Eigen::VectorXf::Zero(this->_x_vector_.size() + 2));
    // call inverse observational model ot change the cone position to the car frame
    Eigen::Vector2f landmark_absolute = this->_observation_model_->inverse_observation_model(
        this->_x_vector_, ObservationData(new_features[i](0), new_features[i](1),
                                          common_lib::competition_logic::Color::BLUE));
    this->_x_vector_(this->_x_vector_.size() - 2) = landmark_absolute(0);
    this->_x_vector_(this->_x_vector_.size() - 1) = landmark_absolute(1);

    Eigen::MatrixXf new_p_matrix =
        Eigen::MatrixXf::Zero(this->_p_matrix_.rows() + 2, this->_p_matrix_.cols() + 2);
    new_p_matrix.block(0, 0, this->_p_matrix_.rows(), this->_p_matrix_.cols()) = this->_p_matrix_;
    this->_p_matrix_ = new_p_matrix;

    // Get Jacobians
    Eigen::MatrixXf Gv = _observation_model_->get_gv(
        _x_vector_, ObservationData(new_features[i](0), new_features[i](1),
                                    common_lib::competition_logic::Color::BLUE));

    Eigen::MatrixXf Gz = _observation_model_->get_gz(
        _x_vector_, ObservationData(new_features[i](0), new_features[i](1),
                                    common_lib::competition_logic::Color::BLUE));

    // Get R (measurement noise covariance)
    Eigen::Matrix2f R = _observation_model_->get_observation_noise_covariance_matrix();

    Eigen::MatrixXf GvT = Gv.transpose();
    Eigen::MatrixXf GzT = Gz.transpose();

    Eigen::MatrixXf P_submatrix = this->_p_matrix_.block(0, 0, 3, 3);

    Eigen::MatrixXf new_submatrix = Gv * P_submatrix * GvT + Gz * R * GzT;

    this->_p_matrix_.block(length, length, 2, 2) = new_submatrix;

    P_submatrix = this->_p_matrix_.block(0, 0, 3, 3);
    new_submatrix = Gv * P_submatrix;
    this->_p_matrix_.block(length, 0, 2, 3) = new_submatrix;

    P_submatrix = this->_p_matrix_.block(length, 0, 2, 3);
    new_submatrix = P_submatrix.transpose();
    this->_p_matrix_.block(0, length, 3, 2) = new_submatrix;

    if (length > 6) {
      this->_p_matrix_.block(length, 6, 2, length - 6) =
          Gv * this->_p_matrix_.block(0, 6, 3, length - 6);
      this->_p_matrix_.block(6, length, length - 6, 2) =
          (this->_p_matrix_.block(length, 6, 2, length - 6)).transpose();
    }
  }
  RCLCPP_DEBUG(rclcpp::get_logger("ekf_state_est"), "STATE AFTER CORRECT %f %f %f %f %f %f",
               this->_x_vector_(0), this->_x_vector_(1), this->_x_vector_(2), this->_x_vector_(3),
               this->_x_vector_(4), this->_x_vector_(5));
}
Eigen::MatrixXf ExtendedKalmanFilter::get_kalman_gain(const Eigen::MatrixXf &h_matrix,
                                                      const Eigen::MatrixXf &p_matrix,
                                                      const Eigen::MatrixXf &q_matrix) const {
  Eigen::MatrixXf s_matrix = h_matrix * p_matrix * h_matrix.transpose() + q_matrix;

  if ((s_matrix.array() == 0).all()) {
    return Eigen::MatrixXf::Zero(p_matrix.rows(), h_matrix.rows());
  }
  Eigen::MatrixXf kalman_gain_matrix = p_matrix * h_matrix.transpose() * s_matrix.inverse();

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
