#include <chrono>
#include <fstream>

#include "gtest/gtest.h"
#include "include/inspection_ros.hpp"
#include "rclcpp/rclcpp.hpp"

using testing::Eq;

/**
 * @brief round to n decimal places
 * 
 * @param number number to round
 * @param n number of decimal places
 * @return double with n decimal places
 */
double round(double number, int n) {
    number *= pow(10, n);
    number = round(number);
    return number/pow(10, n);
}

/**
 * @brief test steering output
 * 
 */
TEST(STEERING, steering1) {
    InspectionFunctions *new_inspection = new InspectionFunctions(
    3.14159265358979323846264338327950288/6.0, 1.0, 2.0, 4.0, 0.254, 0.5, 26, false);
    for (float i = 0; i < 26; i = i + 0.1) {
        double steering = new_inspection->calculate_steering(i);
        EXPECT_FLOAT_EQ(steering,
    sin(i*2*M_PI/new_inspection->turning_period)*new_inspection->max_angle);
    }
}

/**
 * @brief test steering output. Constants in file
 * 
 */
TEST(STEERING, steering2) {
    InspectionFunctions *new_inspection = new InspectionFunctions();
    for (float i = 0; i < 26; i = i + 0.1) {
        double steering = new_inspection->calculate_steering(i);
        EXPECT_FLOAT_EQ(steering,
    sin(i*2*M_PI/new_inspection->turning_period)*new_inspection->max_angle);
    }
}


/**
 * @brief test torque output. Initial speed greater than target. Constants in file
 * 
 */
TEST(TORQUE, torque1) {
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
TEST(TORQUE, torque2) {
    InspectionFunctions *new_inspection = new InspectionFunctions(
    3.14159265358979323846264338327950288/6.0, 1.0, 2.0, 4.0, 0.254, 0.25, 26, false);
    double initial_speed = -4;
    // i represents time
    for (float i = 0; i < 26; i = i+ 0.1) {
        initial_speed += 0.1 * (new_inspection->calculate_torque(initial_speed));
    }
    EXPECT_EQ(round(initial_speed), 1);
}

/**
 * @brief test torque output. Initial speed similar to ideal speed. Constants in file
 * 
 */
TEST(TORQUE, torque3) {
    InspectionFunctions *new_inspection = new InspectionFunctions();
    double initial_speed = 1.1;
    // i represents time
    for (float i = 0; i < 26; i = i+ 0.1) {
        initial_speed += 0.1 * (new_inspection->calculate_torque(initial_speed));
    }
    EXPECT_EQ(round(initial_speed), 1);
}

/**
 * @brief test oscilating the speed
 * 
 */
TEST(TORQUE, torque4) {
    double max_angle = 3.14159265358979323846264338327950288/6.0, ideal_s = 5.0,
    maximum_speed = 5.0, turn_time = 0, wheel_radius = 0.254, Gain = 0.5, stop_time = 30;
    InspectionFunctions *new_inspection = new InspectionFunctions(
    max_angle, ideal_s, maximum_speed, turn_time, wheel_radius, Gain, stop_time, true);
    double initial_speed = 0, max_speed = -1, min_speed = 1000;
    int count = 0;
    // i represents time
    for (float i = 0; i < stop_time; i = i+ 0.1) {
        if (initial_speed > max_speed) {
            max_speed = initial_speed;
        }
        if (initial_speed < min_speed) {
            min_speed = initial_speed;
        }
        initial_speed += 0.1 * (new_inspection->calculate_torque(initial_speed));
        double current_ideal = new_inspection -> ideal_speed;
        new_inspection -> redefine_ideal_speed(initial_speed);

        if (current_ideal != new_inspection -> ideal_speed) {
            count += 1; // count how many times the ideal speed changes
        }
    }
    delete new_inspection;
    EXPECT_EQ(count, 4); // test if the ideal speed changes the right number of times
    EXPECT_EQ(round(min_speed, 1), 0.0);
    EXPECT_DOUBLE_EQ(round(max_speed, 2), 4.8);
}

TEST(TORQUE, torque5) {
    double max_angle = 3.14159265358979323846264338327950288/6.0, ideal_s = 1.0,
    maximum_speed = 1.0, turn_time = 0, wheel_radius = 0.254, Gain = 0.5, stop_time = 26;
    InspectionFunctions *new_inspection = new InspectionFunctions(
    max_angle, ideal_s, maximum_speed, turn_time, wheel_radius, Gain, stop_time, true);
    double initial_speed = 0, max_speed = -1, min_speed = 1000;
    int count = 0;
    for (float i = 0; i < stop_time; i = i+ 0.1) {
        if (initial_speed > max_speed) {
            max_speed = initial_speed;
        }
        if (initial_speed < min_speed) {
            min_speed = initial_speed;
        }
        initial_speed += 0.1 * (new_inspection->calculate_torque(initial_speed));
        double current_ideal = new_inspection -> ideal_speed;
        new_inspection -> redefine_ideal_speed(initial_speed);

        if (current_ideal != new_inspection -> ideal_speed) {
            count += 1; // count how many times the ideal speed changes
        }
    }
    delete new_inspection;
    EXPECT_EQ(count, 9); // test if the ideal speed changes the right number of times
    EXPECT_EQ(round(min_speed, 1), 0.0);
    EXPECT_DOUBLE_EQ(round(max_speed, 2), 0.81);
}

/**
 * @brief test conversion from rpm to velocity with constants defined directly
 * 
 */
TEST(CONVERSION, conversion1) {
    InspectionFunctions *new_inspection = new InspectionFunctions(
    3.14159265358979323846264338327950288/6.0, 1.0, 2.0, 4.0, 0.254, 0.5, 26, false);
    double rpm = 15;
    EXPECT_DOUBLE_EQ(0.39898226700590372, new_inspection->rpm_to_speed(rpm));
}

/**
 * @brief test conversion from rpm to velocity with constants defined in file
 * 
 */
TEST(CONVERSION, conversion2) {
    InspectionFunctions *new_inspection = new InspectionFunctions();
    double rpm = 15;
    EXPECT_DOUBLE_EQ(0.39898226700590372, new_inspection->rpm_to_speed(rpm));
}

