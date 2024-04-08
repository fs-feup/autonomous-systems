
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "speed_est/se_node.hpp"

/**
 * @brief Main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  VehicleState *vehicle_state = new VehicleState();
  vehicle_state->last_update = std::chrono::high_resolution_clock::now();
  MotionUpdate *motion_update = new MotionUpdate();
  motion_update->last_update = std::chrono::high_resolution_clock::now();
  ConeMap *track_map = new ConeMap();       // Map to publish
  ConeMap *perception_map = new ConeMap();  // Map from perception

  Eigen::Matrix2f Q = Eigen::Matrix2f::Zero();
  Q(0, 0) = 0.3;
  Q(1, 1) = 0.3;
  Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  R(0, 0) = 0.8;
  R(1, 1) = 0.8;
  R(2, 2) = 0.8;
  R(3, 3) = 0.8;
  R(4, 4) = 0.8;
  MotionModel *motion_model = new NormalVelocityModel(R);
  ObservationModel observation_model = ObservationModel(Q);

  ExtendedKalmanFilter *ekf = new ExtendedKalmanFilter(*motion_model, observation_model);

  bool use_odometry = true;

  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);

  auto speed_est = std::make_shared<SENode>(ekf, perception_map, motion_update, track_map,
                                            vehicle_state, use_odometry);
  rclcpp::spin(speed_est);
  rclcpp::shutdown();

  return 0;
}
