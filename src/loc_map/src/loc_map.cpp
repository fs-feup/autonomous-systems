#include <cstdio>

#include "loc_map/lm_publisher.hpp"
#include "loc_map/lm_subscriber.hpp"
#include "rclcpp/rclcpp.hpp"
#include "kalman_filter/ekf.hpp"

/**
 * @brief Main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv) {
  Pose *vehicle_localization = new Pose();
  Map *track_map = new Map();

  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LMSubscriber>(track_map));
  rclcpp::spin(std::make_shared<LMPublisher>(vehicle_localization, track_map));
  rclcpp::shutdown();

  return 0;
}
