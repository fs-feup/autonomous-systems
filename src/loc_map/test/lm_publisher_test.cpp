#include "loc_map/lm_publisher.hpp"

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Test Subscriber class
 *
 */
class TestSubscriber : public rclcpp::Node {
  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr _localization_subscription;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr _mapping_subscription;
  std::vector<custom_interfaces::msg::ConeArray> _mapping_messages;
  std::vector<custom_interfaces::msg::Pose> _localization_messages;
  int callback_count = 0;

  /**
   * @brief Check if the callback has been called 10 times and increase the counter
   *
   */
  void _check_callback_count() {
    this->callback_count++;
    if (this->callback_count >= 10) {
      rclcpp::shutdown();
    }
  }

  /**
   * @brief Callback for receiving message from localization topic
   *
   * @param msg
   */
  void _localization_callback(const custom_interfaces::msg::Pose::SharedPtr msg) {
    this->_check_callback_count();
    this->_localization_messages.push_back(*msg);
  }

  /**
   * @brief Callback for receiving message from mapping topic
   *
   * @param msg
   */
  void _mapping_callback(const custom_interfaces::msg::ConeArray::SharedPtr msg) {
    this->_check_callback_count();
    this->_mapping_messages.push_back(*msg);
  }

 public:
  /**
   * @brief Construct a new Test Subscriber object
   *
   */
  TestSubscriber() : Node("test_subscriber") {
    _localization_subscription = this->create_subscription<custom_interfaces::msg::Pose>(
        "vehicle_localization", 10,
        std::bind(&TestSubscriber::_localization_callback, this, std::placeholders::_1));
    _mapping_subscription = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "track_map", 10,
        std::bind(&TestSubscriber::_mapping_callback, this, std::placeholders::_1));
  }

  /**
   * @brief Get the mapping messages object from the subscriber
   *
   * @return std::vector<custom_interfaces::msg::ConeArray>
   */
  std::vector<custom_interfaces::msg::ConeArray> get_mapping_messages() {
    return this->_mapping_messages;
  }

  /**
   * @brief Get the localization messages object from the subscriber
   *
   * @return std::vector<custom_interfaces::msg::Pose>
   */
  std::vector<custom_interfaces::msg::Pose> get_localization_messages() {
    return this->_localization_messages;
  }
};

/**
 * @brief TEST for LMPublisher class
 * tests if the publisher node is publishing the messages
 * and if they are in the correct format and correct topics
 *
 */
TEST(LM_PUBLISHER_TEST_SUITE, PUBLISHER_INTEGRATION_TEST) {
  // Data
  VehicleState *vehicle_state = new VehicleState();
  Map *track_map = new Map();
  track_map->map.insert({{1, 2}, colors::yellow});
  track_map->map.insert({{1, 4}, colors::yellow});
  track_map->map.insert({{1, 6}, colors::yellow});

  rclcpp::init(0, nullptr);

  // Nodes
  auto tester = std::make_shared<TestSubscriber>();
  auto publisher = std::make_shared<LMPublisher>(track_map, vehicle_state);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(tester);
  executor.add_node(publisher);
  executor.spin();
  rclcpp::shutdown();

  EXPECT_GE((int)tester->get_mapping_messages().size(), 3);
  EXPECT_GE((int)tester->get_localization_messages().size(), 3);

  for (auto msg : tester->get_localization_messages()) {
    EXPECT_TRUE(msg.position.x < 10000 && msg.position.x >= -10000);
    EXPECT_TRUE(msg.position.y < 10000 && msg.position.y >= -10000);
    EXPECT_TRUE(msg.theta < 360 && msg.theta >= 0);
  }

  for (auto msg : tester->get_mapping_messages()) {
    for (auto cone : msg.cone_array) {
      EXPECT_TRUE(cone.position.x < 10000 && cone.position.x >= -10000);
      EXPECT_TRUE(cone.position.y < 10000 && cone.position.y >= -10000);
      EXPECT_TRUE(cone.color == colors::color_names[colors::blue] ||
                  cone.color == colors::color_names[colors::yellow]);
    }
  }
}
