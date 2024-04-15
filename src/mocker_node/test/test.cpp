#include <chrono>
#include <fstream>
#include <set>

#include "gtest/gtest.h"
#include "planning/planning_mock.hpp"
#include "rclcpp/rclcpp.hpp"

// Test case for valid input
TEST(GTruthFromFileTest, ValidInput) {
    std::string mockFileContent("header\n-1.0,0,3.1416\n4.0,5.0,6.0\n7.5,-8.5,9.8\n");
    std::istringstream mockFileStream(mockFileContent);

    custom_interfaces::msg::PathPointArray result = gtruth_fromfile(mockFileStream);

    // Uncomment the following lines to see each value in case of bugs
    // std::cout << result.pathpoint_array[0].x << " , " << result.pathpoint_array[0].y
    // << " , " << result.pathpoint_array[0].v << std::endl;
    // std::cout << result.pathpoint_array[1].x << " , " << result.pathpoint_array[1].y
    // << " , " << result.pathpoint_array[1].v << std::endl;
    // std::cout << result.pathpoint_array[2].x << " , " << result.pathpoint_array[2].y
    // << " , " << result.pathpoint_array[2].v << std::endl;

    ASSERT_EQ(result.pathpoint_array.size(), 3); // Three points should be added
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
