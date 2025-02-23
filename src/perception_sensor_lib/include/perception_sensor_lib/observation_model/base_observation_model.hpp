#pragma once

#include <Eigen/Dense>
#include <vector>
class ObservationModel {
public:
  ObservationModel() = default;
  virtual ~ObservationModel() = default;

  /**
   * @brief transform landmarks' positions from global frame to the car's frame
   *
   * @param state vector with the car's position and orientation, followed by the landmark positions
   * {car_x, car_y, car_theta, x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @param matched_landmarks indexes of the x coordinate of landmarks in the state vector that were
   * observed in a specific measurement
   * @return Eigen::VectorXd predicted observations {x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   */
  virtual Eigen::VectorXd observation_model(const Eigen::VectorXd& state,
                                            const std::vector<int> matched_landmarks) const;

  /**
   * @brief transforms the landmarks' coordinates from the car's frame
   * to the global frame
   *
   * @param state used to get the car's pose
   * @param observations observed landmarks in the car's frame in form {x_cone_1, y_cone_1,
   * x_cone_2, y_cone_2, ...}
   * @return Eigen::VectorXd landmarks in the global frame in form {x_cone_1, y_cone_1, x_cone_2,
   * y_cone_2, ...}
   */
  virtual Eigen::VectorXd inverse_observation_model(const Eigen::VectorXd& state,
                                                    const Eigen::VectorXd& observations) const;

  /**
   * @brief jacobian of the observation model
   *
   * @param state vector with the car's position and orientation, followed by the landmark positions
   * {car_x, car_y, car_theta, x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @param matched_landmarks indexes of the x coordinate of landmarks in the state vector that were
   * observed in a specific measurement
   * @return Eigen::MatrixXd jacobian of the observation prediction
   */
  virtual Eigen::MatrixXd observation_model_jacobian(
      const Eigen::VectorXd& state, const std::vector<int>& matched_landmarks) const;

  /**
   * @brief get the Gv matrix used in the state augmentation function, calculated as the jacobian of
   * the inverse observation model with respect to the car's position and orientation
   *
   * @param state car's position and orientation, followed by the landmarks in the form {car_x,
   * car_y, car_theta, x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @param new_landmarks new landmarks in the form {x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @return Eigen::MatrixXd Gv matrix of size (2 * num_new_landmarks, 3)
   */
  virtual Eigen::MatrixXd inverse_observation_model_jacobian_pose(
      const Eigen::VectorXd& state, const Eigen::VectorXd& new_landmarks) const;

  /**
   * @brief get the Gz matrix used in the state augmentation function of the EKF, calculated as the
   * jacobian of the inverse observation model with respect to the new landmarks positions (in the
   * car's frame)
   *
   * @param state car's position and orientation, followed by the landmarks in the form {car_x,
   * car_y, car_theta, x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @param new_landmarks new landmarks in the form {x_cone_1, y_cone_1, x_cone_2, y_cone_2, ...}
   * @return Eigen::MatrixXd Gz matrix of size (2 * num_new_landmarks, 2 * num_new_landmarks)
   */
  virtual Eigen::MatrixXd inverse_observation_model_jacobian_landmarks(
      const Eigen::VectorXd& state, const Eigen::VectorXd& new_landmarks) const;
};