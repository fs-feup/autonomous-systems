#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char *argv[]) {
  rclcpp::init(0, nullptr);
  ::testing::InitGoogleTest(&argc, argv);
  int a = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return a;
}