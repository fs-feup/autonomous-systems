#include <cstdio>

#include "loc_map/lm_ekf_node.hpp"
#include "loc_map/lm_publisher.hpp"
#include "loc_map/lm_subscriber.hpp"
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
  ImuUpdate *imu_update = new ImuUpdate();
  imu_update->last_update = std::chrono::high_resolution_clock::now();
  Map *track_map = new Map();      // Map to publish
  Map *predicted_map = new Map();  // Map from perception
  MotionModel *motion_model = new ImuVelocityModel();
  ObservationModel observation_model = ObservationModel();
  ExtendedKalmanFilter *ekf =
      new ExtendedKalmanFilter(Eigen::MatrixXf::Zero(3, 3), Eigen::MatrixXf::Zero(3, 3),
                               vehicle_state, track_map, imu_update, predicted_map, *motion_model,
                               observation_model);  // TODO(marhcouto): put non zero noise matrixes

  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);

  auto subscriber = std::make_shared<LMSubscriber>(predicted_map, imu_update);
  auto publisher = std::make_shared<LMPublisher>(track_map, vehicle_state);
  auto ekf_node = std::make_shared<EKFNode>(
      ekf);  // TODO(marhcouto): check if this is the best distribution of nodes

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(subscriber);
  executor.add_node(publisher);
  executor.add_node(ekf_node);

  while (rclcpp::ok()) {
    executor.spin_some();
  }

  rclcpp::shutdown();

  return 0;
}
