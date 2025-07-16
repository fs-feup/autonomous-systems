#include "stanley/stanley.hpp"

#include <gtest/gtest.h>

#include "common_lib/structures/position.hpp"
#include "filters/lpf.hpp"
#include "node_/control_parameters.hpp"

using namespace common_lib::structures;

class StanleyTestFixture : public ::testing::Test {
protected:
  std::shared_ptr<Filter> lpf;
  ControlParameters params;
  std::unique_ptr<Stanley> controller;

  void SetUp() override {
    params.stanley_k_ = 4.0;
    params.stanley_epsilon_ = 0.001;
    lpf = std::make_shared<LowPassFilter>(1.0, 0.0);
    controller = std::make_unique<Stanley>(lpf, params);
  }
};

TEST_F(StanleyTestFixture, HeadingErrorZero) {
  LateralControlInput input;
  input.global_cg_position = Position{0, 0};
  input.closest_point = Position{1, 0};
  input.next_closest_point = Position{2, 0};
  input.yaw = 0.0;
  input.velocity = 1.0;

  double path_yaw = std::atan2(input.next_closest_point.y - input.closest_point.y,
                               input.next_closest_point.x - input.closest_point.x);
  double heading_error = controller->normalize_angle(path_yaw - input.yaw);
  EXPECT_NEAR(heading_error, 0.0, 1e-6);
}

TEST_F(StanleyTestFixture, CrossTrackErrorZero) {
  LateralControlInput input;
  input.global_cg_position = Position{1, 0};
  input.closest_point = Position{1, 0};
  input.next_closest_point = Position{2, 0};
  input.yaw = 0.0;
  input.velocity = 1.0;

  double path_yaw = std::atan2(input.next_closest_point.y - input.closest_point.y,
                               input.next_closest_point.x - input.closest_point.x);
  double dx = input.global_cg_position.x - input.closest_point.x;
  double dy = input.global_cg_position.y - input.closest_point.y;
  double cross_track_error = std::sin(path_yaw) * dx - std::cos(path_yaw) * dy;
  EXPECT_NEAR(cross_track_error, 0.0, 1e-6);
}

TEST_F(StanleyTestFixture, SteeringZeroError) {
  LateralControlInput input;
  input.global_cg_position = Position{0, 0};
  input.closest_point = Position{1, 0};
  input.next_closest_point = Position{2, 0};
  input.yaw = 0.0;
  input.velocity = 1.0;

  double steering = controller->steering_control_law(input);
  EXPECT_NEAR(steering, 0.0, 1e-6);
}

TEST_F(StanleyTestFixture, SteeringWithHeadingErrorOnly) {
  LateralControlInput input;
  input.global_cg_position = Position{1, 0};
  input.closest_point = Position{1, 0};
  input.next_closest_point = Position{2, 0.1};  // Path_yaw = atan2(0.1, 1) ≈ 0.0997
  input.yaw = 0.0;
  input.velocity = 2.0;

  // Expected: heading_error ≈ 0.0997, cross_track_error = 0
  double steering = controller->steering_control_law(input);
  EXPECT_NEAR(steering, 0.0997, 0.01);
}

TEST_F(StanleyTestFixture, SteeringWithHeadingAndCrossTrackError) {
  LateralControlInput input;
  input.global_cg_position = Position{0, 1};
  input.closest_point = Position{1, 0};
  input.next_closest_point = Position{2, 0};  // Path_yaw = atan2(0,1) = 0
  input.yaw = 0.0;
  input.velocity = 2.0;

  // Expected: heading_error = 0, cross_track_error = -1 (since path_yaw = 0)
  // cte_term = atan2(4.0 * -1, 2.0 + 0.001) ≈ atan2(-4.0, 2.001) ≈ -1.106
  // steering = heading_error + cte_term = 0 + (-1.106) = -1.106
  double steering = controller->steering_control_law(input);
  EXPECT_NEAR(steering, MIN_STEERING_ANGLE, 0.01);
}