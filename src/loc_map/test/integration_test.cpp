#include "gtest/gtest.h"
#include "loc_map/lm_node.hpp"
#include "rclcpp/rclcpp.hpp"

// /**
//  * @brief Test Subscriber class
//  *
//  */
// class TestSubscriber : public rclcpp::Node {
//   rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr _localization_subscription;
//   rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr _mapping_subscription;
//   rclcpp::TimerBase::SharedPtr _timer;
//   std::vector<custom_interfaces::msg::ConeArray> _mapping_messages;
//   std::vector<custom_interfaces::msg::Pose> _localization_messages;
//   int callback_count = 0;
//   int timer_count = 0;

//   /**
//    * @brief Check if the callback has been called 5 times and increase the counter
//    *
//    */
//   void _check_callback_count() {
//     this->callback_count++;
//     if (this->callback_count >= 5 || this->timer_count >= 10) {
//       rclcpp::shutdown();
//     }
//   }

//   /**
//    * @brief Callback for the timer, to increase the timer count for the execution time limit
//    *
//    */
//   void _timer_callback() { this->timer_count++; }

//   /**
//    * @brief Callback for receiving message from localization topic
//    *
//    * @param msg
//    */
//   void _localization_callback(const custom_interfaces::msg::Pose::SharedPtr msg) {
//     this->_check_callback_count();
//     this->_localization_messages.push_back(*msg);
//   }

//   /**
//    * @brief Callback for receiving message from mapping topic
//    *
//    * @param msg
//    */
//   void _mapping_callback(const custom_interfaces::msg::ConeArray::SharedPtr msg) {
//     this->_check_callback_count();
//     this->_mapping_messages.push_back(*msg);
//   }

//  public:
//   /**
//    * @brief Construct a new Test Subscriber object
//    *
//    */
//   TestSubscriber() : Node("test_subscriber") {
//     _localization_subscription = this->create_subscription<custom_interfaces::msg::Pose>(
//         "vehicle_localization", 10,
//         std::bind(&TestSubscriber::_localization_callback, this, std::placeholders::_1));
//     _mapping_subscription = this->create_subscription<custom_interfaces::msg::ConeArray>(
//         "track_map", 10,
//         std::bind(&TestSubscriber::_mapping_callback, this, std::placeholders::_1));
//     _timer = this->create_wall_timer(std::chrono::milliseconds(1000),
//                                      std::bind(&TestSubscriber::_check_callback_count, this));
//   }

//   /**
//    * @brief Get the mapping messages object from the subscriber
//    *
//    * @return std::vector<custom_interfaces::msg::ConeArray>
//    */
//   std::vector<custom_interfaces::msg::ConeArray> get_mapping_messages() {
//     return this->_mapping_messages;
//   }

//   /**
//    * @brief Get the localization messages object from the subscriber
//    *
//    * @return std::vector<custom_interfaces::msg::Pose>
//    */
//   std::vector<custom_interfaces::msg::Pose> get_localization_messages() {
//     return this->_localization_messages;
//   }
// };

/**
 * @brief TEST for LMNode class
 * tests if the node is publishing the messages
 * and if they are in the correct format and correct topics
 *
 */
TEST(LM_PUBLISH_TEST_SUITE,
     PUBLISH_INTEGRATION_TEST) {  // TODO(marhcouto): implement good integration test
  
  // FROM MAIN
  VehicleState *vehicle_state = new VehicleState();
  vehicle_state->last_update = std::chrono::high_resolution_clock::now();
  MotionUpdate *motion_update = new MotionUpdate();
  motion_update->last_update = std::chrono::high_resolution_clock::now();
  ConeMap *track_map = new ConeMap();       // Map to publish
  ConeMap *perception_map = new ConeMap();  // Map from perception

  Eigen::Matrix2f Q = Eigen::Matrix2f::Zero();
  Q(0, 0) = 0.3;
  Q(1, 1) = 0.3;
  Eigen::MatrixXf R = Eigen::Matrix3f::Zero();
  R(0, 0) = 0.8;
  R(1, 1) = 0.8;
  R(2, 2) = 0.8;
  MotionModel *motion_model = new NormalVelocityModel(R);
  ObservationModel observation_model = ObservationModel(Q);

  ExtendedKalmanFilter *ekf = new ExtendedKalmanFilter(*motion_model, observation_model);

  bool use_odometry = true;
  
  
  // /FROM MAIN
  rclcpp::init(0, nullptr);
  auto receiver_publisher_mock = rclcpp::Node::make_shared("cone_array_mock_publisher"); //test node, publishes and receive results from locmap too

  auto cone_array_msg = std::make_shared<custom_interfaces::msg::ConeArray>();

  auto cones_publisher = receiver_publisher_mock->create_publisher<custom_interfaces::msg::ConeArray>("perception/cone_coordinates", 10);

  custom_interfaces::msg::Pose received_pose;//pose from locmap
  custom_interfaces::msg::ConeArray received_track_map;//track_map from loc map

  

  auto localization_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::Pose>(
    "vehicle_localization", 10,
    [&received_pose](const custom_interfaces::msg::Pose::SharedPtr msg) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n LOCALIZATION RECEIVED \n\n");
      received_pose = *msg;
    });//subscribe to localization topic, get the the pose from locmap

  auto map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
    "track_map", 10,
    [&received_track_map](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
      received_track_map = *msg;
    });//subscribe to track_map topic, get the the map from locmap
  
  custom_interfaces::msg::Cone cone_to_send;
  //custom_interfaces::msg::Cone *cone_to_send;
  custom_interfaces::msg::Point2d point;
  //point.set__x(1);
  //point.set__y(2);
  point.x=1;
  point.y=2;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n POINT X:%d  POINT Y:%d   \n\n", point.x, point.y );
  cone_to_send.set__position(point);
  //cone_to_send = cone_to_send.set__position(point);
  //cone_to_send = cone_to_send.set__color("yellow"); //colors::yellow;//created a cone
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n PUSH BACK TO CONE MSG/msg \n\n");

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n CONE COLOR:%d  CONE X:%d  CONE Y:%d   \n\n",cone_to_send.color, cone_to_send.position.x, cone_to_send.position.y);
  cone_array_msg->cone_array.push_back(cone_to_send);//filled my array of cones message with the new cone
  

  
  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) != RCUTILS_RET_OK) {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info
  

  // auto publisher =
  //     std::make_shared<LMNode>(nullptr, nullptr, nullptr, track_map, vehicle_state, true)
  
  auto lm_node = std::make_shared<LMNode>(ekf, perception_map, motion_update, track_map,
                                          vehicle_state, use_odometry);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n GOT HERE BEFORE PUBLISH \n\n");
  cones_publisher->publish(*cone_array_msg);//send the message
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n GOT HERE AFTER PUBLISH \n\n");
  executor.spin();
  rclcpp::shutdown();
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n GOT HERE AFTER ROS SHUT DOWN \n\n");
  EXPECT_GE(received_track_map.cone_array.size(), 1);
  // EXPECT_GE((int)tester->get_localization_messages().size(), 3);

  // for (auto msg : tester->get_localization_messages()) {
  //   EXPECT_EQ(msg.position.x, 0);
  //   EXPECT_EQ(msg.position.y, 0);
  //   EXPECT_EQ(msg.theta, 0);
  //   EXPECT_EQ(msg.velocity, 0);
  //   EXPECT_EQ(msg.steering_angle, 0);
  // }

  // for (auto msg : tester->get_mapping_messages()) {
  //   for (auto cone : msg.cone_array) {
  //     EXPECT_EQ(cone.position.x, 1);
  //     EXPECT_TRUE(cone.position.y <= 6 && cone.position.y >= 2);
  //     EXPECT_TRUE(cone.color == colors::color_names[colors::blue] ||
  //                 cone.color == colors::color_names[colors::yellow]);
  //   }
  // }
}
