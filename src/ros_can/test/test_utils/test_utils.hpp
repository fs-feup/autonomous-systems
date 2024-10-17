#pragma once

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "custom_interfaces/msg/control_command.hpp"
#include "utils/constants.hpp"
#include "utils/utils.hpp"

/**
 * @brief Matcher to compare the value pointed by a void pointer to a given
 * unsigned char value.
 * @param value The expected unsigned char value.
 */
MATCHER_P(PointeeAsChar, value, "") {
  return *(static_cast<unsigned char *>(arg)) == value;
}

/**
 * @class RosCanTest
 * @brief A test fixture for testing the RosCan class. Initializes the roscan
 * and test node, a wrapper for the mock canlib and an example of a
 * control_command_.
 */
class RosCanTest : public ::testing::Test {
protected:
  /**
   * @brief Set up the test fixture.
   * This function is called before each test is run.
   */
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    test_node_ = std::make_shared<rclcpp::Node>("test_node");

    mock_can_lib_wrapper_ = std::make_shared<MockCanLibWrapper>();
    ros_can_ = std::make_shared<RosCan>(mock_can_lib_wrapper_);

    control_command_ =
        std::make_shared<custom_interfaces::msg::ControlCommand>();
    control_command_->throttle = 0;
    control_command_->steering = 0.1;
    ros_can_->go_signal_ = true;
  }

  /**
   * @brief Tear down the test fixture.
   * This function is called after each test is run.
   */
  void TearDown() override {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  /**
   * @brief helper function to avoid repeated code when testing the control
   * callback calls.
   * @param throttle The throttle value.
   * @param steering The steering value.
   * @param expectedCalls number o0f calls expect for each canWrite function.
   */
  void prepare_control_publish_values(float throttle, float steering,
                                   int expectedCalls) {
    control_command_->throttle = throttle;
    control_command_->steering = steering;
    long steering_id = STEERING_COMMAND_CUBEM_ID;
    EXPECT_CALL(
        *mock_can_lib_wrapper_,
        canWrite(testing::_, steering_id, testing::_, testing::_, testing::_))
        .Times(expectedCalls);

    long throttle_id = BAMO_COMMAND_ID;
    EXPECT_CALL(
        *mock_can_lib_wrapper_,
        canWrite(testing::_, throttle_id, testing::_, testing::_, testing::_))
        .Times(expectedCalls);
  }

  /**
   * @brief Submits request to service and checks that the canWrite function is
   * called with the appropriate msg, used for emergency and mission finished.
   */
  void test_service_call(const std::string &service_name,
                         unsigned char expectedData) {
    // Create a client for the service
    auto client =
        test_node_->create_client<std_srvs::srv::Trigger>(service_name);

    // Wait for the service to be available(optional)
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

    // Create a request for the service
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    long can_id = AS_CU_NODE_ID;

    EXPECT_CALL(*mock_can_lib_wrapper_,
                canRead(testing::_, testing::_, testing::_, testing::_,
                        testing::_, testing::_))
        .WillRepeatedly(testing::Return(canERR_NOMSG));
    EXPECT_CALL(*mock_can_lib_wrapper_,
                canWrite(testing::_, can_id, PointeeAsChar(expectedData),
                         testing::_, testing::_))
        .Times(1)
        .WillOnce(testing::Return(canOK));

    // Call the service
    auto future = client->async_send_request(request);

    // Spin the node for the service call to proceed
    rclcpp::spin_some(ros_can_);
  }

  std::map<std::string, std::string> topics_ = {
      {"emergency", "/as_msgs/emergency"},
      {"mission_finished", "/as_msgs/mission_finished"},
      {"controls", "/as_msgs/controls"},
      {"status", "operational_status_"},
      {"right_rear", "rrRPM"}};

  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Publisher<custom_interfaces::msg::ControlCommand>::SharedPtr
      control_command_publisher_;
  std::shared_ptr<MockCanLibWrapper> mock_can_lib_wrapper_;
  std::shared_ptr<RosCan> ros_can_;
  std::shared_ptr<custom_interfaces::msg::ControlCommand> control_command_;
};

/**
 * @brief Matcher to compare the value pointed by a void pointer to a given
 * double value.
 * @param value The expected double value.
 */
MATCHER_P(PointeeAsDouble, value, "") {
  return *(static_cast<double *>(arg)) == value;
}

/**
 * @brief Check if two doubles are approximately equal.
 * @param a The first double.
 * @param b The second double.
 * @param tolerance The tolerance for the comparison.
 * @return True if the doubles are approximately equal, false otherwise.
 */
bool is_approx_equal(double a, double b, double tolerance) {
  return fabs(a - b) <= tolerance;
}

/**
 * @brief Get the angle from void* passed to the function.
 * @param steering_requestData The request data void*.
 * @return The angle.
 */
float get_angle_from_request_data(void *steering_requestData) {
  char *buffer = static_cast<char *>(steering_requestData);
  int converted_angle = 0;
  for (int i = 0; i < 4; i++) {
    converted_angle |= (static_cast<unsigned char>(buffer[i]) << (8 * (3 - i)));
  }
  float degree_angle = static_cast<float>(converted_angle) / 10000;
  float angle = static_cast<float>(degree_angle * M_PI / 180);
  return angle;
}

/**
 * @brief Matcher to compare the angle obtained from a void pointer to a given
 * expected angle.
 * @param expected_angle The expected angle in radians.
 */
MATCHER_P(PointeeAsAngleEqualTo, expected_angle, "") {
  float angle = get_angle_from_request_data(const_cast<void *>(arg));
  double comp_angle = 0;
  transform_steering_angle_command(expected_angle, comp_angle);
  return is_approx_equal(comp_angle, angle, 0.001);
}

/**
 * @brief Matcher to compare the value pointed by a void pointer to a given
 * unsigned char value.
 * @param expected_throttle_value_ros The expected throttle value in ROS.
 */
MATCHER_P(PointeeAsThrottleValueEqualTo, expected_throttle_value_ros, "") {
  auto *buffer_throttle = static_cast<unsigned char *>(const_cast<void *>(arg));
  int throttle_command = (buffer_throttle[2] << 8) | buffer_throttle[1];
  double actual_throttle_value_ros =
      static_cast<double>(throttle_command) / BAMOCAR_MAX_SCALE;
  return is_approx_equal(actual_throttle_value_ros, expected_throttle_value_ros,
                         0.001);
}

/**
 * @brief Action to set the value pointed by the third argument (arg2) to a
 * given unsigned char array.
 * @param value The unsigned char array to be copied.
 */
ACTION_P(SetArg2ToUnsignedChar, value) {
  auto *dest = static_cast<unsigned char *>(arg2);
  std::copy(value, value + 8, dest);
}
