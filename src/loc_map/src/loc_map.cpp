#include <cstdio>

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
  Localization *vehicle_localization = new Localization();
  Map *track_map = new Map();

  (void)argc;
  (void)argv;
  rclcpp::init(argc, argv);

  auto subscriber = std::make_shared<LMSubscriber>(track_map);
  auto publisher = std::make_shared<LMPublisher>(vehicle_localization, track_map);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(subscriber);
  executor.add_node(publisher);

  while (rclcpp::ok()) {
    executor.spin_some();
  }

  rclcpp::shutdown();

  return 0;
}
