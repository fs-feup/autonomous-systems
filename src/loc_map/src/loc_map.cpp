#include <cstdio>

#include "rclcpp/rclcpp.hpp"

#include "loc_map/lm_publisher.hpp"
#include "loc_map/lm_subscriber.hpp"
#include "loc_map/lm_ekf_node.hpp"

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
  Map *track_map = new Map();
  MotionModel *motion_model = new ImuVelocityModel();
  ExtendedKalmanFilter *ekf = new ExtendedKalmanFilter(vehicle_state, track_map, imu_update, *motion_model);

  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);

  auto subscriber = std::make_shared<LMSubscriber>(track_map, imu_update);
  auto publisher = std::make_shared<LMPublisher>(track_map, vehicle_state);
  auto ekf_node = std::make_shared<EKFNode>(ekf);

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
