#include <cstdio>

#include "kalman_filter/ekf.hpp"
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
  VehicleState *state = new VehicleState();
  state->last_update = std::chrono::high_resolution_clock::now();
  Map *track_map = new Map();

  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LMSubscriber>(track_map, state));
  rclcpp::spin(std::make_shared<LMPublisher>(track_map, state));
  rclcpp::shutdown();

  return 0;
}
