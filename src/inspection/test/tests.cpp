#include <chrono>
#include <fstream>

#include "gtest/gtest.h"
#include "include/inspection_ros.hpp"
#include "rclcpp/rclcpp.hpp"

using testing::Eq;

/**
 * @brief test steering output
 * 
 */
TEST(STEERING, steering) {
    InspectionFunctions *new_inspection = new InspectionFunctions();
    for (float i = 0; i < 26; i = i + 0.1) {
        double steering = new_inspection->calculate_steering(i);
        EXPECT_FLOAT_EQ(steering,
    sin(i*2*M_PI/new_inspection->turning_period)*new_inspection->max_angle);
    }
}

/**
 * @brief test torque output. Initial speed greater than target
 * 
 */
TEST(TORQUE, torque) {;
    InspectionFunctions *new_inspection = new InspectionFunctions();
    double initial_speed = 5;
    // i represents time
    for (float i = 0; i < 26; i = i+ 0.1) {
        initial_speed += 0.1 * (new_inspection->calculate_torque(initial_speed));
    }
    EXPECT_EQ(round(initial_speed), 1);
}

/**
 * @brief test torque output. Initial speed smaller than target.
 * 
 */
TEST(TORQUE, torque2) {;
    InspectionFunctions *new_inspection = new InspectionFunctions();
    double initial_speed = -4;
    // i represents time
    for (float i = 0; i < 26; i = i+ 0.1) {
        initial_speed += 0.1 * (new_inspection->calculate_torque(initial_speed));
    }
    EXPECT_EQ(round(initial_speed), 1);
}

/**
 * @brief test torque output. Initial speed similar to ideal speed.
 * 
 */
TEST(TORQUE, torque3) {;
    InspectionFunctions *new_inspection = new InspectionFunctions();
    double initial_speed = 1.1;
    // i represents time
    for (float i = 0; i < 26; i = i+ 0.1) {
        initial_speed += 0.1 * (new_inspection->calculate_torque(initial_speed));
    }
    EXPECT_EQ(round(initial_speed), 1);
}


/**
 * @brief test conversion from rpm to velocity
 * 
 */
TEST(CONVERSION, conversion) {
    InspectionFunctions *new_inspection = new InspectionFunctions();
    double rpm = 15;
    EXPECT_DOUBLE_EQ(0.39898226700590372, new_inspection->rpm_to_speed(rpm));
}

