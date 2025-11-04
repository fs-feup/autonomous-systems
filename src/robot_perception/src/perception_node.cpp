#include "robot_perception/perception_node.hpp"
#include <sstream>
#include <algorithm>

namespace robot_perception
{

PerceptionNode::PerceptionNode()
: Node("perception_node"),
  map_rows_(0),
  map_cols_(0),
  robot_x_(0.0),
  robot_y_(0.0),
  map_received_(false),
  status_received_(false)
{
  // Declare and get parameter
  this->declare_parameter("perception_radius", 5.0);
  perception_radius_ = this->get_parameter("perception_radius").as_double();
  
  // Create subscribers
  map_sub_ = this->create_subscription<std_msgs::msg::String>(
    "robot/map", 10,
    std::bind(&PerceptionNode::mapCallback, this, std::placeholders::_1));
  
  status_sub_ = this->create_subscription<custom_interfaces::msg::RobotStatus>(
    "robot/status", 10,
    std::bind(&PerceptionNode::statusCallback, this, std::placeholders::_1));
  
  // Create publishers
  perception_pub_ = this->create_publisher<custom_interfaces::msg::PerceptionData>(
    "perception/objects", 10);
  
  local_map_pub_ = this->create_publisher<custom_interfaces::msg::LocalMap>(
    "perception/local_map", 10);
  
  RCLCPP_INFO(this->get_logger(), 
    "Perception node initialized with radius: %.1f", perception_radius_);
}

void PerceptionNode::mapCallback(const std_msgs::msg::String::SharedPtr msg)
{
  extractMapGrid(msg->data);
  map_received_ = true;
  
  // Process perception if we have both map and status
  if (map_received_ && status_received_) {
    processPerception();
  }
}

void PerceptionNode::statusCallback(const custom_interfaces::msg::RobotStatus::SharedPtr msg)
{
  robot_x_ = msg->position.x;
  robot_y_ = msg->position.y;
  status_received_ = true;
  
  // Process perception if we have both map and status
  if (map_received_ && status_received_) {
    processPerception();
  }
}

void PerceptionNode::extractMapGrid(const std::string& map_str)
{
  map_grid_.clear();
  std::istringstream iss(map_str);
  std::string line;
  
  while (std::getline(iss, line)) {
    if (!line.empty()) {
      map_grid_.push_back(line);
    }
  }
  
  map_rows_ = map_grid_.size();
  map_cols_ = map_rows_ > 0 ? map_grid_[0].size() : 0;
}

void PerceptionNode::processPerception()
{
  // Generate and publish perception data (list of objects)
  auto perception_msg = custom_interfaces::msg::PerceptionData();
  perception_msg.perception_radius = perception_radius_;
  perception_msg.objects = detectObjects();
  
  perception_pub_->publish(perception_msg);
  
  // Generate and publish local map (5x5 grid)
  auto local_map_msg = generateLocalMap();
  local_map_pub_->publish(local_map_msg);
}

std::vector<custom_interfaces::msg::PerceivedObject> PerceptionNode::detectObjects()
{
  std::vector<custom_interfaces::msg::PerceivedObject> objects;
  
  // Iterate through the map and find objects within perception radius
  for (int row = 0; row < map_rows_; ++row) {
    for (int col = 0; col < map_cols_; ++col) {
      char cell = getMapCell(row, col);
      
      // Skip empty floor and robot position
      if (cell == '0' || cell == 'S') {
        continue;
      }
      
      // Convert grid coordinates to world coordinates
      double world_x = static_cast<double>(col);
      double world_y = static_cast<double>(map_rows_ - 1 - row);
      
      // Check if within perception radius
      if (isWithinPerceptionRadius(world_x, world_y)) {
        // Convert to relative coordinates
        double rel_x, rel_y;
        globalToRelative(world_x, world_y, rel_x, rel_y);
        
        // Create perceived object
        custom_interfaces::msg::PerceivedObject obj;
        obj.object_type = objectTypeFromChar(cell);
        obj.object_id = std::string(1, cell);
        obj.relative_position.x = rel_x;
        obj.relative_position.y = rel_y;
        
        objects.push_back(obj);
      }
    }
  }
  
  return objects;
}

custom_interfaces::msg::LocalMap PerceptionNode::generateLocalMap()
{
  custom_interfaces::msg::LocalMap local_map;
  local_map.header.stamp = this->now();
  local_map.header.frame_id = "robot";
  local_map.size = 5;
  local_map.robot_global_position.x = robot_x_;
  local_map.robot_global_position.y = robot_y_;
  
  // Calculate robot's grid position
  int robot_row = map_rows_ - 1 - static_cast<int>(std::round(robot_y_));
  int robot_col = static_cast<int>(std::round(robot_x_));
  
  // Generate 5x5 grid centered on robot
  for (int dr = -2; dr <= 2; ++dr) {
    for (int dc = -2; dc <= 2; ++dc) {
      int row = robot_row + dr;
      int col = robot_col + dc;
      
      char cell;
      if (dr == 0 && dc == 0) {
        // Center is always the robot
        cell = 'R';
      } else if (row < 0 || row >= map_rows_ || col < 0 || col >= map_cols_) {
        // Out of bounds
        cell = '$';
      } else {
        // Get actual map cell
        cell = getMapCell(row, col);
        // Replace 'S' (start position) with floor if robot is not there
        if (cell == 'S') {
          cell = '0';
        }
      }
      
      local_map.grid.push_back(std::string(1, cell));
    }
  }
  
  return local_map;
}

bool PerceptionNode::isWithinPerceptionRadius(double x, double y) const
{
  double dx = x - robot_x_;
  double dy = y - robot_y_;
  double distance = std::sqrt(dx * dx + dy * dy);
  return distance <= perception_radius_;
}

void PerceptionNode::globalToRelative(double global_x, double global_y, 
                                       double& rel_x, double& rel_y) const
{
  rel_x = global_x - robot_x_;
  rel_y = global_y - robot_y_;
}

char PerceptionNode::getMapCell(int row, int col) const
{
  if (row < 0 || row >= map_rows_ || col < 0 || col >= static_cast<int>(map_grid_[row].size())) {
    return '$';  // Out of bounds
  }
  return map_grid_[row][col];
}

std::string PerceptionNode::objectTypeFromChar(char c) const
{
  if (c >= '1' && c <= '4') {
    return "box";
  } else if (c >= 'A' && c <= 'D') {
    return "shelf";
  } else if (c == '9') {
    return "wall";
  } else if (c == '0') {
    return "floor";
  } else {
    return "unknown";
  }
}

}  // namespace robot_perception

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_perception::PerceptionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}