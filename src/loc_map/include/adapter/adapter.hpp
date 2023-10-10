#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_

#include <string>

#include "eufs_msgs/msg/can_state.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "eufs_msgs/srv/set_can_state.hpp"
#include "fs_msgs/msg/finished_signal.hpp"
#include "fs_msgs/msg/go_signal.hpp"
#include "fs_msgs/msg/wheel_states.hpp"
#include "loc_map/data_structures.hpp"
#include "loc_map/lm_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class LMNode;
/**
 * @brief Enum for the existing modes
 *
 */
enum Mode { eufs, fsds, ads_dv };
/**
 * @brief Class that handles the communication between the loc_map node and the
 * other nodes in the system according to the selected mode
 *
 */
class Adapter {
  LMNode* node;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _eufs_imu_subscription;
  rclcpp::Subscription<eufs_msgs::msg::CanState>::SharedPtr _eufs_mission_state_subscription;
  rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr
      _eufs_wheel_speeds_subscription;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _fs_imu_subscription;
  rclcpp::Subscription<fs_msgs::msg::WheelStates>::SharedPtr _fs_wheel_speeds_subscription;

  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_mission_state_client_;
  rclcpp::Client<eufs_msgs::srv::SetCanState>::SharedPtr eufs_ebs_client_;

  rclcpp::Publisher<fs_msgs::msg::FinishedSignal>::SharedPtr fsds_ebs_publisher_;

  /**
   * @brief Function that inits the subscribers and client connections for
   * the eufs mode
   *
   */
  void eufs_init();

  /**
   * @brief Function that inits the subscribers and publishers for
   * the fsds mode
   *
   */
  void fsds_init();

  /**
   * @brief Function that inits the subscribers for the ads_dv mode
   * NOTE (JoaoAMarinho):(currently not implemented and may be
   * substituted by the eufs mode)
   *
   */
  void ads_dv_init();

  /**
   * @brief Function that parses the message sent from the ros IMU topic
   * and calls the respective node's function to update the motion
   *
   * @param msg Message sent from the ros IMU topic
   */
  void imu_subscription_callback(const sensor_msgs::msg::Imu msg);

  /**
   * @brief Mission state callback for the eufs mode, which updates
   * the mission state in the loc_map node
   *
   * @param msg Message sent from the ros_can state topic
   */
  void eufs_mission_state_callback(const eufs_msgs::msg::CanState msg);

  /**
   * @brief Function that parses the odometry data from the wheels for the
   * eufs mode and calls the respective node's function to update the motion
   *
   * @param msg Message sent from the ros_can wheel speeds topic
   */
  void eufs_wheel_speeds_subscription_callback(const eufs_msgs::msg::WheelSpeedsStamped msg);

  /**
   * @brief Mission state callback for the fsds mode, which updates
   * the mission state in the loc_map node
   *
   * @param msg Message sent from the fsds go signal topic
   */
  void fsds_mission_state_callback(const fs_msgs::msg::GoSignal msg);

  /**
   * @brief Function that parses the odometry data from the wheels for the
   * fsds mode and calls the respective node's function to update the motion
   *
   * @param msg Message sent from the fsds wheel states topic
   */
  void fsds_wheel_speeds_subscription_callback(const fs_msgs::msg::WheelStates msg);

 public:
  /**
   * @brief LM Adapter constructor declaration
   *
   * @param mode Mode relative to the test environment, default is eufs
   * @param loc_map_node Pointer to the loc_map node
   */
  Adapter(Mode mode, LMNode* loc_map_node);
  explicit Adapter(LMNode* loc_map_node) : Adapter(Mode::eufs, loc_map_node) {}
};

#endif  // SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADAPTER_HPP_
