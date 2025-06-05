#include "pure_pursuit/pp.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include "node_/node_control.hpp"

using namespace common_lib::structures;

class PurePursuitTestFixture : public ::testing::Test {
protected:
    std::shared_ptr<Filter> lpf_no_effect;
    std::shared_ptr<PurePursuit> lat_controller_;

    void SetUp() override {
        // LPF with alpha=1.0 means no filtering
        lpf_no_effect = std::make_shared<LowPassFilter>(1.0, 0.0);
        lat_controller_ = std::make_shared<PurePursuit>(lpf_no_effect);
    }
};

/**
 * @brief Test PurePursuit - calculate_alpha()
 * typical angle - path point left
 */
TEST_F(PurePursuitTestFixture, Test_calculate_alpha_1) {
    Position vehicle_cg = Position(5, 4.46);
    Position vehicle_rear_wheel = Position(6, 2);
    Position lookahead_point = Position(1, 4);
    double rear_wheel_2_c_g = 2.655484889;

    double alpha = lat_controller_->calculate_alpha(vehicle_rear_wheel, vehicle_cg, lookahead_point,
                                                   rear_wheel_2_c_g);
    EXPECT_NEAR(0.804, alpha, 0.001);
}

/**
 * @brief Test PurePursuit - calculate_alpha()
 * typical angle - path point right
 */
TEST_F(PurePursuitTestFixture, Test_calculate_alpha_2) {
    Position vehicle_cg = Position(3, 3);
    Position vehicle_rear_wheel = Position(3, 5);
    Position lookahead_point = Position(5, 1);
    double rear_wheel_2_c_g = 2;

    double alpha = lat_controller_->calculate_alpha(vehicle_rear_wheel, vehicle_cg, lookahead_point,
                                                   rear_wheel_2_c_g);

    EXPECT_NEAR(0.463, alpha, 0.001);
}

/**
 * @brief Test PurePursuit - calculate_alpha()
 * 90 degrees angle
 */
TEST_F(PurePursuitTestFixture, Test_calculate_alpha_3) {
    Position vehicle_cg = Position(3, 3);
    Position vehicle_rear_wheel = Position(3, 5);
    Position lookahead_point = Position(5, 5);
    double rear_wheel_2_c_g = 2;

    double alpha = lat_controller_->calculate_alpha(vehicle_rear_wheel, vehicle_cg, lookahead_point,
                                                   rear_wheel_2_c_g);

    EXPECT_NEAR(1.570, alpha, 0.001);
}

/**
 * @brief Test PurePursuit - calculate_alpha()
 * 0 degrees angle
 */
TEST_F(PurePursuitTestFixture, Test_calculate_alpha_4) {
    Position vehicle_cg = Position(3, 3);
    Position vehicle_rear_wheel = Position(3, 5);
    Position lookahead_point = Position(3, 0);
    double rear_wheel_2_c_g = 2;

    double alpha = lat_controller_->calculate_alpha(vehicle_rear_wheel, vehicle_cg, lookahead_point,
                                                   rear_wheel_2_c_g);

    EXPECT_NEAR(0, alpha, 0.001);
}

/**
 * @brief Test PurePursuit - calculate_alpha()
 * 180 degrees angle
 */
TEST_F(PurePursuitTestFixture, Test_calculate_alpha_5) {
    Position vehicle_cg = Position(3, 3);
    Position vehicle_rear_wheel = Position(3, 5);
    Position lookahead_point = Position(3, 7);
    double rear_wheel_2_c_g = 2;

    double alpha = lat_controller_->calculate_alpha(vehicle_rear_wheel, vehicle_cg, lookahead_point,
                                                   rear_wheel_2_c_g);

    EXPECT_NEAR(3.141, alpha, 0.001);
}

/**
 * @brief Test PurePursuit - pp_steering_control_law()
 */
TEST_F(PurePursuitTestFixture, Test_pp_steering_control_law_1) {
    Position cg = Position(5, 4.46);
    Position rear_axis = Position(6, 2);
    Position lookahead_point = Position(1, 4);
    double dist_cg_2_rear_axis = 2.655484889;

    double steering_cmd = lat_controller_->pp_steering_control_law(
        rear_axis, cg, lookahead_point, dist_cg_2_rear_axis);

    //  Alpha: 0.804189
    //  ld_: 5.38516
    EXPECT_NEAR(0.381, steering_cmd, 0.001);
}