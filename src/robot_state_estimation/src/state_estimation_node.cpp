// Fixed State Estimation Node: incremental map building

#include <cmath>
#include <custom_interfaces/msg/perception_data.hpp>
#include <custom_interfaces/msg/robot_status.hpp>
#include <custom_interfaces/msg/state_estimation_map.hpp>
#include <custom_interfaces/msg/state_estimation_position.hpp>
#include <custom_interfaces/msg/state_estimation_debug.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class StateEstimationNode : public rclcpp::Node {
public:
  StateEstimationNode()
      : Node("state_estimation"),
        grid_size_(101),
        center_x_(50),
        center_y_(50),
        robot_grid_x_(50),
        robot_grid_y_(50),
        robot_relative_to_S_x_(0),
        robot_relative_to_S_y_(0),
        robot_world_x_(0),
        robot_world_y_(0),
        corner_00_x_(0),
        corner_00_y_(0),
        corner_00_found_(false),
        last_update_time_(this->now()) {
    using std::placeholders::_1;

    // Initialize grid with '?' (unknown)
    grid_.resize(grid_size_);
    for (int y = 0; y < grid_size_; ++y) {
      grid_[y] = std::string(grid_size_, '?');
    }

    // marcar robot starting position with 'S'
    grid_[robot_grid_y_][robot_grid_x_] = 'S';

    status_sub_ = this->create_subscription<custom_interfaces::msg::RobotStatus>(
        "robot/status", rclcpp::QoS(10), std::bind(&StateEstimationNode::onStatus, this, _1));

    perception_sub_ = this->create_subscription<custom_interfaces::msg::PerceptionData>(
        "perception/objects", rclcpp::QoS(10),
        std::bind(&StateEstimationNode::onPerception, this, _1));

    pos_pub_ = this->create_publisher<custom_interfaces::msg::StateEstimationPosition>(
        "state_estimation/position", rclcpp::QoS(10));
    map_pub_ = this->create_publisher<custom_interfaces::msg::StateEstimationMap>(
        "state_estimation/map", rclcpp::QoS(10));
      
    debug_pub_ = this->create_publisher<custom_interfaces::msg::StateEstimationDebug>(
        "state_estimation/debug", rclcpp::QoS(10));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&StateEstimationNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "State Estimation node started (grid %dx%d)", grid_size_,
                grid_size_);
  }

private:

  void publishDebugInfo() {
    custom_interfaces::msg::StateEstimationDebug debug_msg;
    debug_msg.robot_grid_x = robot_grid_x_;
    debug_msg.robot_grid_y = robot_grid_y_;
    debug_msg.robot_relative_to_s_x = robot_relative_to_S_x_;
    debug_msg.robot_relative_to_s_y = robot_relative_to_S_y_;
    debug_msg.robot_world_x = robot_world_x_;
    debug_msg.robot_world_y = robot_world_y_;
    debug_msg.corner_00_x = corner_00_x_;
    debug_msg.corner_00_y = corner_00_y_;
    debug_msg.corner_00_found = corner_00_found_;
    
    debug_pub_->publish(debug_msg);
  }

//done??
void onStatus(const custom_interfaces::msg::RobotStatus::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "On Status was called");
  // Update robot's world position based on velocity
  
  int dist_x = msg->velocity.x;
  int dist_y = msg->velocity.y;

  // Update if there's any movement (changed condition)
  if (dist_x != 0 || dist_y != 0) {

    // Update robot grid position 
    if (isValidGridPos(robot_grid_x_ + dist_x, robot_grid_y_ - dist_y)) {
      
      // Store previous robot position
      int prev_robot_grid_x = robot_grid_x_;
      int prev_robot_grid_y = robot_grid_y_;
      
      // Update robot grid position 
      robot_grid_x_ += dist_x;
      robot_grid_y_ -= dist_y;

      if(robot_world_x_!=0 && robot_world_y_ != 0){
        robot_world_x_ += dist_x;
        robot_world_y_ += dist_y;
      }

      // Update relative position to S
      robot_relative_to_S_x_ += dist_x;
      robot_relative_to_S_y_ += dist_y;

      // Mark previous robot position as floor ('0'), but preserve 'S' at start
      if (grid_[prev_robot_grid_y][prev_robot_grid_x] == 'R') {
        grid_[prev_robot_grid_y][prev_robot_grid_x] = '0';
      }
      
      // Mark current robot position as 'R', preserve 'S' if at start
      if (grid_[robot_grid_y_][robot_grid_x_] == '?' || grid_[robot_grid_y_][robot_grid_x_] == '0'){
        grid_[robot_grid_y_][robot_grid_x_] = 'R';
      }
      
      publishDebugInfo();
    }
  }
}

//done?
  void onPerception(const custom_interfaces::msg::PerceptionData::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "On Perception was called");
    double perception_radius = msg->perception_radius;

    // First pass -> mark all positions within perception radius as floor ('0') unless they're already marked with an object
    markPerceptionRadiusAsFloor(robot_grid_x_, robot_grid_y_, perception_radius); //correto

    // Second pass: place all detected objects on the grid
    for (const auto& obj : msg->objects) {
      const std::string& type = obj.object_type;
      const std::string& id = obj.object_id;
      int rel_x = obj.relative_position.x;
      int rel_y = obj.relative_position.y;

      // Calculate grid position relative to current robot position and relative position to S
      int obj_grid_x = robot_grid_x_ + rel_x;
      int obj_grid_y = robot_grid_y_ - rel_y;

      if (!isValidGridPos(obj_grid_x, obj_grid_y)) {
        continue;
      }

      // Place object using its ID character, or map type to character if ID is empty
      char obj_char = '?';
      if (!id.empty()) {
        obj_char = id[0];
      } else {
        obj_char = getCharForType(type);
      }

      // Overwrite floor with object, but preserve 'S' at start position
      if (grid_[obj_grid_y][obj_grid_x] == '0' || grid_[obj_grid_y][obj_grid_x] == '?') {
        grid_[obj_grid_y][obj_grid_x] = obj_char;
      }
    }
    
    // Third pass: detect bottom-left corner if not already found
    if (!corner_00_found_) {
      detectBottomLeftCorner();
    }
    publishDebugInfo();
  }

//done?
void detectBottomLeftCorner() {
  // Scan the grid to find a bottom-left corner pattern:
  // A corner is where we have walls forming an L-shape
  // Bottom-left corner: wall at (x,y), wall to the RIGHT at (x+1,y), wall ABOVE at (x,y-1)
  // Remember: in grid array, lower y index = higher up visually

  for (int y = 1; y < grid_size_ - 1; ++y) {  // Start from y=1 so we can check y-1
    for (int x = 0; x < grid_size_ - 1; ++x) {
      if (grid_[y][x] != '9') continue;  // current cell must be a wall

      bool wall_right = (grid_[y][x + 1] == '9');
      bool wall_above = (grid_[y - 1][x] == '9');  // FIXED: y-1 not y+1 (lower index = higher up)

      // Check the L-shape pattern for bottom-left corner
      if (wall_right && wall_above) {
        corner_00_found_ = true;
        corner_00_x_ = x;
        corner_00_y_ = y;

        double corner_relative_to_S_x_ = (x - center_x_);
        double corner_relative_to_S_y_ = (center_y_ - y);  // FIXED: inverted for correct world coordinates

        RCLCPP_INFO(this->get_logger(),
                    "Found bottom-left corner at grid position (%d,%d), relative position to Start (%.1f, %.1f)",
                    corner_00_x_, corner_00_y_, corner_relative_to_S_x_, corner_relative_to_S_y_);

        // mark out-of-bounds to the left and below
        // Left of corner (x < corner_x)
        for (int mx = 0; mx < x; ++mx) {
          for (int my = 0; my < grid_size_; ++my) {
            if (grid_[my][mx] == '?') grid_[my][mx] = 'X';
          }
        }
        // Below corner (y > corner_y, higher index = lower visually)
        for (int my = y + 1; my < grid_size_; ++my) {  // FIXED: y+1 to grid_size_ (below)
          for (int mx = 0; mx < grid_size_; ++mx) {
            if (grid_[my][mx] == '?') grid_[my][mx] = 'X';
          }
        }

        return;
      }
    }
  }
}
  
//done? probabilby need to move the publish to the initializer and change it when found the corner
  void onTimer() {
    // Always publish the map
    custom_interfaces::msg::StateEstimationMap map_msg;
    map_msg.grid = grid_;
    map_pub_->publish(map_msg);

    // Only publish position if bottom-left corner (0,0) has been found
    if (corner_00_found_) {
      // Calculate robot's absolute position
      // The corner is at grid position (corner_00_x_, corner_00_y_)
      // Since corner is at absolute (0,0), robot's absolute position is:
      double robot_world_x_ = corner_00_x_ - robot_grid_x_;
      double robot_world_y_ = corner_00_y_ - robot_grid_y_;

      custom_interfaces::msg::StateEstimationPosition pos_msg;
      pos_msg.x = robot_world_x_;
      pos_msg.y = robot_world_y_;
      pos_pub_->publish(pos_msg);
    }
    publishDebugInfo();
  }

//done
  bool isValidGridPos(int x, int y) const {
    return x >= 0 && x < grid_size_ && y >= 0 && y < grid_size_;
  }

  //done?
  void markPerceptionRadiusAsFloor(int center_x, int center_y, double radius) {
    // marcar todas as celulas dentro do percpetion radius como 0 se nao conhecidas
    int radius_int = static_cast<int>(std::ceil(radius));
    for (int dy = -radius_int; dy <= radius_int; ++dy) {
      for (int dx = -radius_int; dx <= radius_int; ++dx) {
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist <= radius) {
          int x = center_x + dx;
          int y = center_y + dy;
          if (isValidGridPos(x, y)) {
            if (grid_[y][x] == '?') {
              grid_[y][x] = '0';
            }
          }
        }
      }
    }
  }

// need to edit to other characters in other maps
  char getCharForType(const std::string& type) const {
    if (type == "empty" || type == "floor") {
      return '0';
    } else if (type == "box") {
      return '1';
    } else if (type == "wall") {
      return '9';
    } else if (type == "shelf") {
      return 'A';
    }
    return '?';  // Unknown territory
  }

  // --- Members ---
  rclcpp::Subscription<custom_interfaces::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<custom_interfaces::msg::PerceptionData>::SharedPtr perception_sub_;
  rclcpp::Publisher<custom_interfaces::msg::StateEstimationPosition>::SharedPtr pos_pub_;
  rclcpp::Publisher<custom_interfaces::msg::StateEstimationMap>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_interfaces::msg::StateEstimationDebug>::SharedPtr debug_pub_;

  // Grid representation: 101x101, center at (50, 50)
  int grid_size_;
  int center_x_;
  int center_y_;
  int robot_grid_x_;  // Current grid position of robot
  int robot_grid_y_;  // Current grid position of robot
  std::vector<std::string> grid_;

  // Robot relative position compared to starting position
  int robot_relative_to_S_x_;
  int robot_relative_to_S_y_;

  // Robot position in world coordinates -- only completed when corner found
  int robot_world_x_;
  int robot_world_y_;

  // Corner (0,0) detection in the grid
  int corner_00_x_;
  int corner_00_y_;
  bool corner_00_found_;

  rclcpp::Time last_update_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimationNode>());
  rclcpp::shutdown();
  return 0;
}