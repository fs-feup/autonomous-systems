#include "gtest/gtest.h"
#include <fstream>//to write file
#include "loc_map/lm_node.hpp"
#include "rclcpp/rclcpp.hpp"
//#include <unistd.h> //used to get cwd

// /**
//  * @brief Test Subscriber class
//  *
//  */
// class TestSubscriber : public rclcpp::Node {
//   rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr _localization_subscription;
//   rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr _mapping_subscription;
//   //rclcpp::TimerBase::SharedPtr _timer;
//   std::vector<custom_interfaces::msg::ConeArray> _mapping_messages;
//   std::vector<custom_interfaces::msg::Pose> _localization_messages;

//   /**
//    * @brief Callback for receiving message from localization topic
//    *
//    * @param msg
//    */
//   void _localization_callback(const custom_interfaces::msg::Pose::SharedPtr msg) {
//     this->_localization_messages.push_back(*msg);
//   }

//   /**
//    * @brief Callback for receiving message from mapping topic
//    *
//    * @param msg
//    */
//   void _mapping_callback(const custom_interfaces::msg::ConeArray::SharedPtr msg) {
//     //this->_check_callback_count();
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

class ExecTimeTest : public ::testing::Test {
 protected:
  void SetUp() override {
     
  }

  // void TearDown() override {}
};

/**
 * @brief TEST for LMNode class
 * tests if the node is publishing the messages
 * and if they are in the correct format and correct topics
 *
 */

TEST(LM_PUBLISH_TEST_SUITE,
     PUBLISH_INTEGRATION_TEST) {  
  
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

  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) != RCUTILS_RET_OK) {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

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
      

      rclcpp::shutdown();

    });//subscribe to track_map topic, get the the map from locmap
  
  custom_interfaces::msg::Cone cone_to_send;
  cone_to_send.position.x=1;
  cone_to_send.position.y=2;
  cone_to_send.color="yellow_cone"; //colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send);//filled my array of cones message with the new cone
  cone_to_send.position.x=2;
  cone_to_send.position.y=4;
  cone_to_send.color="blue_cone"; //colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send);//filled my array of cones message with the new cone
  cone_to_send.position.x=4;
  cone_to_send.position.y=6;
  cone_to_send.color="yellow_cone"; //colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send);//filled my array of cones message with the new cone
  
  auto lm_node = std::make_shared<LMNode>(ekf, perception_map, motion_update, track_map,
                                          vehicle_state, use_odometry);//complete loc map node
  
  rclcpp::executors::MultiThreadedExecutor executor; // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node);
  auto start_time = std::chrono::high_resolution_clock::now();
  //std::this_thread::sleep_for(std::chrono::seconds(1));
  cones_publisher->publish(*cone_array_msg);//send the cones
  executor.spin();
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %lld ms", duration.count());
  
  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }


  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);//apped
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
  file << "LOC_MAP, all, 3 cones, " << duration.count() <<"\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), 3);
  

}

