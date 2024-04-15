#include <chrono>
#include <fstream>
#include <set>

#include "gtest/gtest.h"
#include "include/planning_mock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/files.hpp"

// Test case for valid input
TEST(GTruthFromFileTest, ValidInput) {
    std::string mockFileContent("header\n1.0,2.0,3.0\n4.0,5.0,6.0\n7.0,8.0,9.0\n");
    std::istringstream mockFileStream(mockFileContent);

    custom_interfaces::msg::PathPointArray result = gtruth_fromfile(mockFileStream);

    ASSERT_EQ(result.pathpoint_array.size(), 3); // Three points should be added
    // Additional assertions for each PathPoint in the result
}
