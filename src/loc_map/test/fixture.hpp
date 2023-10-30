
#include <fstream>  //to write file
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "loc_map/lm_node.hpp"
#include "rclcpp/rclcpp.hpp"
// microsegundos - /100

class ExecTimeTest : public ::testing::Test {
 protected:
  VehicleState *vehicle_state_test;
  MotionUpdate *motion_update_test;
  ConeMap *track_map_test;
  ConeMap *perception_map_test;
  Eigen::Matrix2f Q_test;
  Eigen::MatrixXf R_test;
  MotionModel *motion_model_test;
  ObservationModel observation_model_test = ObservationModel(Q_test);
  ExtendedKalmanFilter *ekf_test;
  bool use_odometry_test;
  int helper;
  std::shared_ptr<rclcpp::Node> receiver_publisher_mock;
  std::shared_ptr<rclcpp::Node> lm_node_test;

  std::shared_ptr<custom_interfaces::msg::ConeArray> cone_array_msg;
  std::shared_ptr<rclcpp::Publisher<custom_interfaces::msg::ConeArray>> cones_publisher;

  custom_interfaces::msg::Pose received_pose;            // pose from locmap
  custom_interfaces::msg::ConeArray received_track_map;  // track_map_test from loc map
  std::shared_ptr<rclcpp::Subscription<custom_interfaces::msg::ConeArray>> map_sub;
  std::shared_ptr<rclcpp::Subscription<custom_interfaces::msg::Pose>> localization_sub;

  std::chrono::_V2::system_clock::time_point start_time;
  std::chrono::microseconds duration;
  std::chrono::_V2::system_clock::time_point end_time;
  custom_interfaces::msg::Cone cone_to_send;
  int helper_1;
  bool done;
  std::string workload;
  void print_to_file() {
    std::ofstream file("../../src/performance/exec_time/loc_map.csv", std::ios::app);  // append

    // Convert the duration from microseconds to milliseconds
    double milliseconds =
        static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(duration).count()) /
        1000.0;

    file << "LOC_MAP, " << workload << ", " << std::fixed << milliseconds << " ms\n";
    file.close();
  }

  void fill_X(int size) {
    for (int i = 3; i <= size; i++) {
      double randomX = (static_cast<double> (rand() / RAND_MAX)) * 50.0;
      double randomY = (static_cast<double> (rand() / RAND_MAX)) * 50.0;

      // Call set_X_y for X and Y values
      ekf_test->set_X_y(i, randomX);
      ekf_test->set_X_y(i + 1, randomY);

      // Call push_to_colors with the current color
      if (i % 2 == 0) {
        ekf_test->push_to_colors(colors::Color::blue);
      } else {
        ekf_test->push_to_colors(colors::Color::yellow);
      }

      // Increment the index by 2
      i++;
    }
  }

  void fill_cone_array(int size) {
    for (int i = 0; i < size; i++) {
      // Generate random x and y coordinates
      int x = rand() % 50;
      int y = rand() % 50;

      // Set the cone position
      cone_to_send.position.x = x;
      cone_to_send.position.y = y;

      // Set the cone color
      if (i % 2 == 0) {
        cone_to_send.color = "yellow_cone";
      } else {
        cone_to_send.color = "blue_cone";
      }

      // Add the cone to the cone array message
      cone_array_msg->cone_array.push_back(cone_to_send);
    }
  }

  void SetUp() override {  // TODO(PedroRomao3) //SetUpTestSuite
    start_time = std::chrono::high_resolution_clock::now();
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    helper = 0;
    helper_1 = 0;
    vehicle_state_test = new VehicleState();
    vehicle_state_test->last_update = std::chrono::high_resolution_clock::now();
    motion_update_test = new MotionUpdate();
    motion_update_test->last_update = std::chrono::high_resolution_clock::now();
    track_map_test = new ConeMap();       // Map to publish
    perception_map_test = new ConeMap();  // Map from perception

    Q_test = Eigen::Matrix2f::Zero();
    Q_test(0, 0) = 0.3;
    Q_test(1, 1) = 0.3;
    R_test = Eigen::Matrix3f::Zero();
    R_test(0, 0) = 0.8;
    R_test(1, 1) = 0.8;
    R_test(2, 2) = 0.8;
    motion_model_test = new NormalVelocityModel(R_test);
    observation_model_test = ObservationModel(Q_test);

    ekf_test = new ExtendedKalmanFilter(*motion_model_test, observation_model_test);
    done = false;
    use_odometry_test = true;

    receiver_publisher_mock = rclcpp::Node::make_shared(
        "cone_array_mock_publisher");  // test node, publishes and receive results from locmap too

    cone_array_msg = std::make_shared<custom_interfaces::msg::ConeArray>();

    cones_publisher = receiver_publisher_mock->create_publisher<custom_interfaces::msg::ConeArray>(
        "perception/cone_coordinates", 10);

    map_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::ConeArray>(
        "track_map", 10, [this](const custom_interfaces::msg::ConeArray::SharedPtr msg) {
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n TRACK RECEIVED \n\n");
          received_track_map = *msg;
          done = true;
        });  // subscribe to track_map topic, get the the map from locmap

    localization_sub = receiver_publisher_mock->create_subscription<custom_interfaces::msg::Pose>(
        "vehicle_localization", 10, [this](const custom_interfaces::msg::Pose::SharedPtr msg) {
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n\n LOCALIZATION RECEIVED \n\n");
          received_pose = *msg;
        });

    lm_node_test = std::make_shared<LMNode>(ekf_test, perception_map_test, motion_update_test,
                                            track_map_test, vehicle_state_test, use_odometry_test);
  }
  void TearDown() override {
    receiver_publisher_mock.reset();
    cones_publisher.reset();
    map_sub.reset();
    localization_sub.reset();
    lm_node_test.reset();  // TearDownTestSuite
    delete vehicle_state_test;
    delete motion_update_test;
    delete track_map_test;
    delete perception_map_test;
    // delete motion_model_test;
    delete ekf_test;

    // executor.cancel();
  }
};
