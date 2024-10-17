#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "canlib_test_wrappers/mock_can_lib_wrapper.hpp"
#include "node/node_ros_can.hpp"
#include "test_utils/test_utils.hpp"
#include "utils/constants.hpp"

/**
 * @test This test case checks if the control callback function correctly writes the steering and
 * throttle commands to the CAN bus. 
 * 
 * Scenario: Steering and Throttle between the limits
 */
TEST_F(RosCanTest, ControlCallback) {
  prepare_control_publish_values(0.3, 0.3, 1);
  ros_can_->control_callback(control_command_);
}

/**
 * @test Test Scenario with throttle below the limits.
 */
TEST_F(RosCanTest, ControlCallbackLowerThrottle) {
  prepare_control_publish_values(-1.1, 0.3, 0);
  ros_can_->control_callback(control_command_);
}

/**
 * @test Test Scenario with throttle above the limits.
 */
TEST_F(RosCanTest, ControlCallbackHigherThrottle) {
  prepare_control_publish_values(1.1, 0.3, 0);
  ros_can_->control_callback(control_command_);
}

/**
 * @test Test Scenario with steering below the limits.
 */
TEST_F(RosCanTest, ControlCallbackLowerSteering) {
  prepare_control_publish_values(0.3, -0.5, 0);
  ros_can_->control_callback(control_command_);
}

/**
 * @test Test Scenario with steering above the limits.
 */
TEST_F(RosCanTest, ControlCallbackHigherSteering) {
  prepare_control_publish_values(0.3, 0.5, 0);
  ros_can_->control_callback(control_command_);
}


/**
 * @test This test represesnts the case where the steering ot the throttle violates the expected limits (Higher Value). 
 * Expected message to not be written to CAN
 * 
 */
TEST_F(RosCanTest, TestOutOfRangeUpperSteeringThrottle) {
  prepare_control_publish_values(1.4, STEERING_UPPER_LIMIT + 1, 0);
  ros_can_->control_callback(control_command_);
}

/**
 * @test This test represesnts the case where the steering ot the throttle violates the expected limits (Lower Value). 
 * Expected message to not be written to CAN
 * 
 */
TEST_F(RosCanTest, TestOutOfRangeLowerSteeringThrottle) {
  prepare_control_publish_values(-0.7, STEERING_LOWER_LIMIT - 1, 0);
  ros_can_->control_callback(control_command_);
}


/**
 * @test This test represents the case where the steering and the throttle violates the expected limits (Higher and Lower Value). 
 * Expected message to not be written to CAN
 */
TEST_F(RosCanTest, TestOutOfRangeSingleSteeringThrottle) {
  prepare_control_publish_values(-0.7, STEERING_UPPER_LIMIT + 1, 0);
  ros_can_->control_callback(control_command_);
}

/**
 * @test This test publishes a command with the ROSCAN node initialized, then we test if the
 * behaviour of the nodes is correct, activate the control callback and correctly write to can the
 * received values.
 */
TEST_F(RosCanTest, PublishControlCommand) {
  control_command_->throttle = 0.5;
  control_command_->steering = 0.1;
  control_command_publisher_ =
      test_node_->create_publisher<custom_interfaces::msg::ControlCommand>(topics_["controls"], 10);

  long steering_id = STEERING_COMMAND_CUBEM_ID;
  EXPECT_CALL(*mock_can_lib_wrapper_,
              canWrite(testing::_, steering_id, PointeeAsAngleEqualTo(control_command_->steering),
                       testing::_, testing::_))
      .Times(1)
      .WillOnce(testing::Return(canOK));

  EXPECT_CALL(*mock_can_lib_wrapper_,
              canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .WillRepeatedly(testing::Return(canERR_NOMSG));

  long throttle_id = BAMO_COMMAND_ID;
  EXPECT_CALL(
      *mock_can_lib_wrapper_,
      canWrite(testing::_, throttle_id, PointeeAsThrottleValueEqualTo(control_command_->throttle),
               testing::_, testing::_))
      .Times(1)
      .WillOnce(testing::Return(canOK));

  control_command_publisher_->publish(*control_command_);

  rclcpp::spin_some(ros_can_);
}

/**
 * @test Checks the request to service emergency and that the canWrite function is
 * called with the appropriate msg.
 */
TEST_F(RosCanTest, EmergencyCallback) { 
  test_service_call("/as_srv/emergency", EMERGENCY_CODE); 
}

/**
 * @test Checks the request to service mission_finished and that the canWrite function is
 * called with the appropriate msg.
 */
TEST_F(RosCanTest, MissionFinishedCallback) {
  test_service_call("/as_srv/mission_finished", MISSION_FINISHED_CODE);
}


/**
 * @test This test checks if the IMU acceleration readings from can are correctly published to the
 * correct ROS TOPIC, the test node is used to subscribe to the respective topic and read the values
 * and verify them.
 */

TEST_F(RosCanTest, TestImuAccPublisher) {
  unsigned char msg[8] = {0x80, 0x00, 0x00, 0x00, 0xFF, 0xFE, 0x00, 0x00};
  long id = IMU_ACC;
  unsigned int dlc = 8;
  unsigned int flag = 0;
  unsigned long time = 0;

  EXPECT_CALL(*mock_can_lib_wrapper_,
              canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(2)
      .WillOnce(testing::DoAll(testing::SetArgPointee<1>(id), SetArg2ToUnsignedChar(msg),
                               testing::SetArgPointee<3>(dlc), testing::SetArgPointee<4>(flag),
                               testing::SetArgPointee<5>(time), testing::Return(canOK)))
      .WillRepeatedly(testing::Return(canERR_NOMSG));
  
  auto imuAcc = test_node_->create_subscription<custom_interfaces::msg::ImuAcceleration>(
      "/vehicle/acceleration", 10, [&](const custom_interfaces::msg::ImuAcceleration::SharedPtr msg) {
        const double tolerance = 0.001;
        EXPECT_NEAR(msg->acc_x, 0.0, tolerance);
        EXPECT_NEAR(msg->acc_y, -64.278, tolerance);
        EXPECT_NEAR(msg->acc_z, 64.274, tolerance);
      });

    ros_can_->can_sniffer();

  rclcpp::spin_some(test_node_);
}

/**
 * @test This test checks if the IMU gyroscope readings from can are correctly published to the
 * correct ROS TOPIC, the test node is used to subscribe to the respective topic and read the values
 * and verify them.
 */
TEST_F(RosCanTest, TestImuGyroPublisher) {
  unsigned char msg[8] = {0x80, 0x00, 0x0A, 0xD0, 0xF5, 0x30, 0x00, 0x00};
  long id = IMU_GYRO;
  unsigned int dlc = 8;
  unsigned int flag = 0;
  unsigned long time = 0;

  EXPECT_CALL(*mock_can_lib_wrapper_,
              canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(2)
      .WillOnce(testing::DoAll(testing::SetArgPointee<1>(id), SetArg2ToUnsignedChar(msg),
                               testing::SetArgPointee<3>(dlc), testing::SetArgPointee<4>(flag),
                               testing::SetArgPointee<5>(time), testing::Return(canOK)))
      .WillRepeatedly(testing::Return(canERR_NOMSG));
  
  auto imuGyro = test_node_->create_subscription<custom_interfaces::msg::YawPitchRoll>(
      "/vehicle/angular_velocity", 10, [&](const custom_interfaces::msg::YawPitchRoll::SharedPtr msg) {
        const double tolerance = 0.001;
        EXPECT_NEAR(msg->yaw, 300.0, tolerance);
        EXPECT_NEAR(msg->pitch, -300.0, tolerance);
        EXPECT_NEAR(msg->roll, 0.0, tolerance);
      });

    ros_can_->can_sniffer();

  rclcpp::spin_some(test_node_);
}

/**
 * @test This test checks if the operational status callback function is called after receiving an
 * operational status message from ROS and if the the callback function correctly sends the
 * operational status signal to can.
 */

TEST_F(RosCanTest, TestCanInterpreterMasterStatusMission) {
  unsigned char msg[8] = {MASTER_AS_MISSION_CODE, 0x02, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
  long id = MASTER_STATUS;
  unsigned int dlc = 8;
  unsigned int flag = 0;
  unsigned long time = 0;

  EXPECT_CALL(*mock_can_lib_wrapper_,
              canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(2)
      .WillOnce(testing::DoAll(testing::SetArgPointee<1>(id), SetArg2ToUnsignedChar(msg),
                               testing::SetArgPointee<3>(dlc), testing::SetArgPointee<4>(flag),
                               testing::SetArgPointee<5>(time), testing::Return(canOK)))
      .WillRepeatedly(testing::Return(canERR_NOMSG));

  auto masterStatusSub = test_node_->create_subscription<custom_interfaces::msg::OperationalStatus>(
      topics_["status"], 10, [&](const custom_interfaces::msg::OperationalStatus::SharedPtr msg) {
        EXPECT_EQ(msg->as_mission, 2);
        EXPECT_EQ(msg->go_signal, 0);
      });

    ros_can_->can_sniffer();

  rclcpp::spin_some(test_node_);
}


/**
 * @test This test case checks if after receiving RR_RPM from the CAN bus, the values are correctly
 * published to the correct ROS TOPIC, the test node is used to subscribe to the respective topic
 * and read the values published and verify them
 */
TEST_F(RosCanTest, TestCanInterpreter_TEENSY_C1_RR_RPM_CODE) {
  unsigned char msg[8] = {TEENSY_C1_RR_RPM_CODE, 0x1F, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00};
  long id = TEENSY_C1;
  unsigned int dlc = 8;
  unsigned int flag = 0;
  unsigned long time = 0;

  EXPECT_CALL(*mock_can_lib_wrapper_,
              canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(2)
      .WillOnce(testing::DoAll(testing::SetArgPointee<1>(id), SetArg2ToUnsignedChar(msg),
                               testing::SetArgPointee<3>(dlc), testing::SetArgPointee<4>(flag),
                               testing::SetArgPointee<5>(time), testing::Return(canOK)))
      .WillRepeatedly(testing::Return(canERR_NOMSG));

  auto rr_rpm_pub_sub = test_node_->create_subscription<custom_interfaces::msg::WheelRPM>(
      topics_["right_rear"], 10, [&](const custom_interfaces::msg::WheelRPM::SharedPtr msg) {
        const double tolerance = 0.0001;
        EXPECT_NEAR(msg->rr_rpm, 164.15, tolerance);
      });

    ros_can_->can_sniffer();

  rclcpp::spin_some(test_node_);
}

/**
 * @test This test case confirms that the car state must be driving to send the steering and
 * throttle commands to the CAN bus.
 */
TEST_F(RosCanTest, TestCarStateMustBeDriving) {
  ros_can_->go_signal_ = false;
  long steering_id = STEERING_COMMAND_CUBEM_ID;
  EXPECT_CALL(*mock_can_lib_wrapper_,
              canWrite(testing::_, steering_id, testing::_, testing::_, testing::_))
      .Times(0);

  long throttle_id = BAMO_COMMAND_ID;
  EXPECT_CALL(*mock_can_lib_wrapper_,
              canWrite(testing::_, throttle_id, testing::_, testing::_, testing::_))
      .Times(0);

    ros_can_->control_callback(control_command_);
}

/**
 * @test This test case confirms that the alive message is sent to the CAN bus every 100ms.
 */

TEST_F(RosCanTest, TestAliveMsgCallback) {
  auto new_node = std::make_shared<RosCan>(mock_can_lib_wrapper_);
  EXPECT_CALL(*mock_can_lib_wrapper_,
              canWrite(testing::_, testing::_, testing::_, testing::_, testing::_))
      .Times(3)
      .WillRepeatedly(testing::Return(canOK));
  EXPECT_CALL(*mock_can_lib_wrapper_,
              canRead(testing::_, testing::_, testing::_, testing::_, testing::_, testing::_))
      .WillRepeatedly(testing::Return(canERR_NOMSG));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(new_node);

  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(395)) {
    executor.spin_some();
  }
}


/**
 * @test This test case checks if the the method correctly changes the wheels steering angle to the
 * actuator steering angle.
 */

TEST(UtilsTest, TransformSteeringAngleCommand) {
  double actuator_steering_angle = 0.0;

  double wheels_steering_angle = 0.1;
  double expected_actuator_steering_angle = 0.50677179486251434;

  int result = transform_steering_angle_command(wheels_steering_angle, actuator_steering_angle);
  EXPECT_EQ(result, 0);
  EXPECT_DOUBLE_EQ(actuator_steering_angle, expected_actuator_steering_angle);
}
