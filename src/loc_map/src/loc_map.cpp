#include <cstdio>

#include "loc_map/lm_node.hpp"
#include "rclcpp/rclcpp.hpp"

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
  Map *track_map = new Map();      // Map to publish
  Map *predicted_map = new Map();  // Map from perception

  Eigen::Matrix2f Q = Eigen::Matrix2f::Zero();
  Q(0, 0) = 0.1;
  Q(1, 1) = 0.1;
  Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  R(0, 0) = 0.1;
  R(1, 1) = 0.1;
  R(2, 2) = 0.1;
  MotionModel *motion_model = new NormalVelocityModel(R);
  ObservationModel observation_model = ObservationModel(Q);

  ExtendedKalmanFilter *ekf = new ExtendedKalmanFilter(*motion_model, observation_model);

  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);

  auto subscriber =
      std::make_shared<LMNode>(ekf, predicted_map, motion_update, track_map, vehicle_state, true);
  // auto publisher = std::make_shared<LMPublisher>(track_map, vehicle_state);
  // auto ekf_node = std::make_shared<EKFNode>(
  //     ekf);  // TODO(marhcouto): check if this is the best distribution of nodes

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(subscriber);
  // executor.add_node(publisher);
  // executor.add_node(ekf_node);

  while (rclcpp::ok()) {
    executor.spin_some();
  }

  rclcpp::shutdown();

  return 0;
}
