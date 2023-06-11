#include "gtest/gtest.h"

#include "kalman_filter/ekf.hpp"
#include "loc_map/lm_subscriber.hpp"
#include "loc_map/data_structures.hpp"

TEST(SLAM_TEST_IUITE, VELOCITY_MODEL_TEST) {

  VehicleState* vehicle_state = new VehicleState();
  ImuUpdate *imu_update = new ImuUpdate();
  Map* track_map = new Map();

  rclcpp::init(0, nullptr);

  auto subscriber = std::make_shared<LMSubscriber>(track_map, imu_update);
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(subscriber);
  while (rclcpp::ok()) {
    executor.spin_some();
  }

  rclcpp::shutdown();

}