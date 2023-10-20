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
  // public:
  // ExecTimeTest() {}
 protected:
  VehicleState *vehicle_state;
  MotionUpdate *motion_update;
  ConeMap *track_map;
  ConeMap *perception_map;
  Eigen::Matrix2f Q;
  Eigen::MatrixXf R;
  MotionModel *motion_model;
  //ObservationModel observation_model;
  ObservationModel observation_model = ObservationModel(Q);
  ExtendedKalmanFilter *ekf;
  bool use_odometry;
  std::shared_ptr<rclcpp::Node> receiver_publisher_mock;
  std::shared_ptr<custom_interfaces::msg::ConeArray> cone_array_msg;
  std::shared_ptr<rclcpp::Publisher<custom_interfaces::msg::ConeArray>> cones_publisher;


  custom_interfaces::msg::Pose received_pose;//pose from locmap
  custom_interfaces::msg::ConeArray received_track_map;//track_map from loc map
  std::shared_ptr<rclcpp::Subscription<custom_interfaces::msg::ConeArray>> map_sub;
  std::shared_ptr<rclcpp::Subscription<custom_interfaces::msg::Pose>> localization_sub;
  
  void SetUp() override {
    rclcpp::init(0, nullptr);
    vehicle_state = new VehicleState();
    vehicle_state->last_update = std::chrono::high_resolution_clock::now();
    motion_update = new MotionUpdate();
    motion_update->last_update = std::chrono::high_resolution_clock::now();
    track_map = new ConeMap();       // Map to publish
    perception_map = new ConeMap();  // Map from perception
    
    Q = Eigen::Matrix2f::Zero();
    Q(0, 0) = 0.3;
    Q(1, 1) = 0.3;
    R = Eigen::Matrix3f::Zero();
    R(0, 0) = 0.8;
    R(1, 1) = 0.8;
    R(2, 2) = 0.8;
    motion_model = new NormalVelocityModel(R);
    observation_model = ObservationModel(Q);

    ekf = new ExtendedKalmanFilter(*motion_model, observation_model);

    use_odometry = true;

    receiver_publisher_mock = rclcpp::Node::make_shared("cone_array_mock_publisher");//test node, publishes and receive results from locmap too

    cone_array_msg = std::make_shared<custom_interfaces::msg::ConeArray>();

    cones_publisher = receiver_publisher_mock->create_publisher<custom_interfaces::msg::ConeArray>("perception/cone_coordinates", 10);

    map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
    "track_map", 10,
    [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
      received_track_map = *msg;
      

      rclcpp::shutdown();

    });//subscribe to track_map topic, get the the map from locmap

    localization_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::Pose>(
    "vehicle_localization", 10,
    [this](const custom_interfaces::msg::Pose::SharedPtr msg) {
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n LOCALIZATION RECEIVED \n\n");
        received_pose = *msg;
    });

    
  }
  void TearDown() override {
    
    if (vehicle_state != nullptr) {
      delete vehicle_state;
      vehicle_state = nullptr;
    }

    if (motion_update != nullptr) {
      delete motion_update;
      motion_update = nullptr;
    }

    if (track_map != nullptr) {
      delete track_map;
      track_map = nullptr;
    }

    if (perception_map != nullptr) {
      delete perception_map;
      perception_map = nullptr;
    }

    if (motion_model != nullptr) {
      //delete motion_model;
      motion_model = nullptr;
    }

    if (ekf != nullptr) {
      delete ekf;
      ekf = nullptr;
    }

    if (receiver_publisher_mock) {
      rclcpp::shutdown();
      receiver_publisher_mock.reset();
    }
    rclcpp::shutdown();
  // Reset other shared pointers and perform additional cleanup as needed
  }

  // void TearDown() override {}
};

/**
 * @brief TEST for LMNode class
 * tests if the node is publishing the messages
 * and if they are in the correct format and correct topics
 *
 */

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_1CONE) {  


  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) != RCUTILS_RET_OK) {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

  
  custom_interfaces::msg::Cone cone_to_send_4;
  cone_to_send_4.position.x=1;
  cone_to_send_4.position.y=2;
  cone_to_send_4.color="yellow_cone"; //colors::yellow;//created a cone
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// BEFORE ADDING FORTH: %ld ///////\n", cone_array_msg->cone_array.size());
  cone_array_msg->cone_array.push_back(cone_to_send_4);//filled my array of cones message with the new cone
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// AFTER ADDING FORTH: %ld ///////\n", cone_array_msg->cone_array.size());
  //complete loc map node
  //std::this_thread::sleep_for(std::chrono::seconds(1));
  auto lm_node1 = std::make_shared<LMNode>(ekf, perception_map, motion_update, track_map, vehicle_state, use_odometry);
  cones_publisher->publish(*cone_array_msg);//send the cones
  rclcpp::executors::MultiThreadedExecutor executor; // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node1);
  auto start_time = std::chrono::high_resolution_clock::now();
  executor.spin();
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());
  
  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }


  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);//apped
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
  file << "LOC_MAP, all, 1 cones, " << duration.count() <<"\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(),(long unsigned int) 1);
  

}

TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_3CONES) {  


  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) != RCUTILS_RET_OK) {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

  
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
  //complete loc map node
  //std::this_thread::sleep_for(std::chrono::seconds(1));
  auto lm_node3 = std::make_shared<LMNode>(ekf, perception_map, motion_update, track_map, vehicle_state, use_odometry);
  cones_publisher->publish(*cone_array_msg);//send the cones
  rclcpp::executors::MultiThreadedExecutor executor; // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node3);
  auto start_time = std::chrono::high_resolution_clock::now();
  executor.spin();
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());
  
  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }


  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);//apped
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
  file << "LOC_MAP, all, 3 cones, " << duration.count() <<"\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(), (long unsigned int) 3);
  

}
TEST_F(ExecTimeTest, PUBLISH_INTEGRATION_TEST_4CONES) {  


  // if (rcutils_logging_set_logger_level("loc_map", RCUTILS_LOG_SEVERITY_ERROR) != RCUTILS_RET_OK) {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error setting logger level");
  // }  // suppress warnings and info

  
  custom_interfaces::msg::Cone cone_to_send_4;
  cone_to_send_4.position.x=1;
  cone_to_send_4.position.y=2;
  cone_to_send_4.color="yellow_cone"; //colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send_4);//filled my array of cones message with the new cone
  cone_to_send_4.position.x=2;
  cone_to_send_4.position.y=4;
  cone_to_send_4.color="blue_cone"; //colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send_4);//filled my array of cones message with the new cone
  cone_to_send_4.position.x=4;
  cone_to_send_4.position.y=6;
  cone_to_send_4.color="yellow_cone"; //colors::yellow;//created a cone
  cone_array_msg->cone_array.push_back(cone_to_send_4);//filled my array of cones message with the new cone
  cone_to_send_4.position.x=6;
  cone_to_send_4.position.y=8;
  cone_to_send_4.color="blue_cone"; //colors::yellow;//created a cone
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// BEFORE ADDING FORTH: %ld ///////\n", cone_array_msg->cone_array.size());
  cone_array_msg->cone_array.push_back(cone_to_send_4);//filled my array of cones message with the new cone
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n/////// AFTER ADDING FORTH: %ld ///////\n", cone_array_msg->cone_array.size());
  //complete loc map node
  //std::this_thread::sleep_for(std::chrono::seconds(1));
  auto lm_node = std::make_shared<LMNode>(ekf, perception_map, motion_update, track_map, vehicle_state, use_odometry);
  cones_publisher->publish(*cone_array_msg);//send the cones
  rclcpp::executors::MultiThreadedExecutor executor; // create executor and add all nodes
  executor.add_node(receiver_publisher_mock);
  executor.add_node(lm_node);
  auto start_time = std::chrono::high_resolution_clock::now();
  executor.spin();
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Execution time: %ld ms", duration.count());
  
  // char cwd[1024];
  // if (getcwd(cwd, sizeof(cwd)) != nullptr) {
  //   //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n///////\nDIR: %s \n///////", cwd);
  // } else {
  //   //err
  // }


  std::ofstream file("../../src/loc_map/test/integration_test.csv", std::ios::app);//apped
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%d", file.is_open());
  file << "LOC_MAP, all, 4 cones, " << duration.count() <<"\n";
  file.close();

  EXPECT_GE(received_track_map.cone_array.size(),(long unsigned int) 4);
  

}

