#include <chrono>
#include <fstream>
#include <set>

#include "gtest/gtest.h"
#include "utils/mocks.hpp"
#include "rclcpp/rclcpp.hpp"

// Test case for valid input
TEST(GTruthFromFileTest, ValidInput) {
  std::string mockFileContent("header asdasd\n-1.0,0,3.1416\n4.0,5.0,6.0\n7.5,-8.5,9.8");
  std::istringstream mockFileStream(mockFileContent);

  custom_interfaces::msg::PathPointArray result = planning_gtruth_fromfile(mockFileStream);

  // Uncomment the following lines to see each value in case of bugs
  // std::cout << result.pathpoint_array[0].x << " , " << result.pathpoint_array[0].y
  // << " , " << result.pathpoint_array[0].v << std::endl;
  // std::cout << result.pathpoint_array[1].x << " , " << result.pathpoint_array[1].y
  // << " , " << result.pathpoint_array[1].v << std::endl;
  // std::cout << result.pathpoint_array[2].x << " , " << result.pathpoint_array[2].y
  // << " , " << result.pathpoint_array[2].v << std::endl;

  ASSERT_EQ(result.pathpoint_array.size(), 3);  // Three points should be added
  EXPECT_FLOAT_EQ(result.pathpoint_array[0].x, -1.0);
  EXPECT_FLOAT_EQ(result.pathpoint_array[0].y, 0.0);
  EXPECT_FLOAT_EQ(result.pathpoint_array[0].v, 3.1416);
  EXPECT_FLOAT_EQ(result.pathpoint_array[1].x, 4.0);
  EXPECT_FLOAT_EQ(result.pathpoint_array[1].y, 5.0);
  EXPECT_FLOAT_EQ(result.pathpoint_array[1].v, 6.0);
  EXPECT_FLOAT_EQ(result.pathpoint_array[2].x, 7.5);
  EXPECT_FLOAT_EQ(result.pathpoint_array[2].y, -8.5);
  EXPECT_FLOAT_EQ(result.pathpoint_array[2].v, 9.8);
}

// Test case for empty input
TEST(GTruthFromFileTest, EmptyInput) {
  std::string mockFileContent("");
  std::istringstream mockFileStream(mockFileContent);

  custom_interfaces::msg::PathPointArray result = planning_gtruth_fromfile(mockFileStream);

  ASSERT_EQ(result.pathpoint_array.size(), 0);  // No points should be added
}

// Test case for invalid input
TEST(GTruthFromFileTest, InvalidInput) {
  std::string mockFileContent("header\n-1.0,0,3.1416\n4.0,5.0,6.0\n7.5,-8.5,9.8\ninvalid");
  std::istringstream mockFileStream(mockFileContent);

  custom_interfaces::msg::PathPointArray result = planning_gtruth_fromfile(mockFileStream);

  ASSERT_EQ(result.pathpoint_array.size(),
            3);  // Three points should be added; the other line should be ignored
}