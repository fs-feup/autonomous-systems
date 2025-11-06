#ifndef ROBOT_PERCEPTION__PERCEPTION_NODE_HPP_
#define ROBOT_PERCEPTION__PERCEPTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_interfaces/msg/robot_status.hpp>
#include <custom_interfaces/msg/perception_data.hpp>
#include <custom_interfaces/msg/local_map.hpp>
#include <custom_interfaces/msg/perceived_object.hpp>

#include <string>
#include <vector>
#include <cmath>

namespace robot_perception
{

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode();

private:
  // Callback functions
  void mapCallback(const std_msgs::msg::String::SharedPtr msg);
  void statusCallback(const custom_interfaces::msg::RobotStatus::SharedPtr msg);
  
  // Processing functions
  void processPerception();
  void extractMapGrid(const std::string& map_str);
  std::vector<custom_interfaces::msg::PerceivedObject> detectObjects();
  custom_interfaces::msg::LocalMap generateLocalMap();
  
  // Helper functions
  bool isWithinPerceptionRadius(double x, double y) const;
  void globalToRelative(double global_x, double global_y, double& rel_x, double& rel_y) const;
  char getMapCell(int row, int col) const;
  std::string objectTypeFromChar(char c) const;
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_sub_;
  rclcpp::Subscription<custom_interfaces::msg::RobotStatus>::SharedPtr status_sub_;
  
  // Publishers
  rclcpp::Publisher<custom_interfaces::msg::PerceptionData>::SharedPtr perception_pub_;
  rclcpp::Publisher<custom_interfaces::msg::LocalMap>::SharedPtr local_map_pub_;
  
  // State variables
  std::vector<std::string> map_grid_;
  int map_rows_;
  int map_cols_;
  double robot_x_;
  double robot_y_;
  bool map_received_;
  bool status_received_;
  
  // Parameters
  double perception_radius_;
};

}  // namespace robot_perception

#endif  // ROBOT_PERCEPTION__PERCEPTION_NODE_HPP_
