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
        corner_00_x_(0),
        corner_00_y_(0),
        corner_00_found_(false),
        last_update_time_(this->now()) {
    using std::placeholders::_1;

    grid_.resize(grid_size_);
    for (int y = 0; y < grid_size_; ++y) {
      grid_[y] = std::string(grid_size_, '?');
    }

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
    debug_msg.corner_00_x = corner_00_x_;
    debug_msg.corner_00_y = corner_00_y_;
    debug_msg.corner_00_found = corner_00_found_;
    
    debug_pub_->publish(debug_msg);
  }

void onStatus(const custom_interfaces::msg::RobotStatus::SharedPtr msg) {
  int dist_x = msg->velocity.x;
  int dist_y = msg->velocity.y;

  if (dist_x != 0 || dist_y != 0) {

    if (isValidGridPos(robot_grid_x_ + dist_x, robot_grid_y_ - dist_y)) {
      
      int prev_robot_grid_x = robot_grid_x_;
      int prev_robot_grid_y = robot_grid_y_;
       
      robot_grid_x_ += dist_x;
      robot_grid_y_ -= dist_y;

      robot_relative_to_S_x_ += dist_x;
      robot_relative_to_S_y_ += dist_y;

      if (grid_[prev_robot_grid_y][prev_robot_grid_x] == 'R') {
        grid_[prev_robot_grid_y][prev_robot_grid_x] = '0';
      }
      
      if (grid_[robot_grid_y_][robot_grid_x_] == '?' || grid_[robot_grid_y_][robot_grid_x_] == '0'){
        grid_[robot_grid_y_][robot_grid_x_] = 'R';
      }
      
      publishDebugInfo();
    }
  }
}

void onPerception(const custom_interfaces::msg::PerceptionData::SharedPtr msg) {
  double perception_radius = msg->perception_radius;

  markPerceptionRadiusAsFloor(robot_grid_x_, robot_grid_y_, perception_radius);

  for (const auto& obj : msg->objects) {
    const std::string& type = obj.object_type;
    const std::string& id = obj.object_id;
    int rel_x = obj.relative_position.x;
    int rel_y = obj.relative_position.y;

    int obj_grid_x = static_cast<int>(std::round(robot_grid_x_ + obj.relative_position.x));
    int obj_grid_y = static_cast<int>(std::round(robot_grid_y_ - obj.relative_position.y));


    if (!isValidGridPos(obj_grid_x, obj_grid_y)) {
      continue;
    }

    char obj_char = '?';
    if (!id.empty()) {
      obj_char = id[0];
    } else {
      obj_char = getCharForType(type);
    }

    if (grid_[obj_grid_y][obj_grid_x] == '0' || grid_[obj_grid_y][obj_grid_x] == '?') {
      grid_[obj_grid_y][obj_grid_x] = obj_char;
    }

    if (obj_char == '9') {
      if (rel_x == 0 && rel_y == 5) {
        detectedUpWall(obj_grid_x, obj_grid_y);
      }
      
      if (rel_x == 0 && rel_y == -5) {
        detectedDownWall(obj_grid_x, obj_grid_y);
      }
      
      if (rel_x == -5 && rel_y == 0) {
        detectedLeftWall(obj_grid_x, obj_grid_y);
      }
      
      if (rel_x == 5 && rel_y == 0) {
        detectedRightWall(obj_grid_x, obj_grid_y);
      }
      
      if (!corner_00_found_) {
        bool wall_right = isValidGridPos(obj_grid_x + 1, obj_grid_y) && 
                         grid_[obj_grid_y][obj_grid_x + 1] == '9';
        bool wall_above = isValidGridPos(obj_grid_x, obj_grid_y - 1) && 
                         grid_[obj_grid_y - 1][obj_grid_x] == '9';
        
        if (wall_right && wall_above) {
          detectedBottomLeftCorner(obj_grid_x, obj_grid_y);
        }
      }
    }
  }
  
  publishDebugInfo();
}

void detectedUpWall(int wall_grid_x, int wall_grid_y) {
  RCLCPP_INFO(this->get_logger(), "Detected UP wall at grid (%d,%d)", wall_grid_x, wall_grid_y);
  
  for (int my = 0; my < wall_grid_y; ++my) {
    for (int mx = 0; mx < grid_size_; ++mx) {
      if (grid_[my][mx] == '?') {
        grid_[my][mx] = 'X';
      }
    }
  }
}

void detectedDownWall(int wall_grid_x, int wall_grid_y) {
  RCLCPP_INFO(this->get_logger(), "Detected DOWN wall at grid (%d,%d)", wall_grid_x, wall_grid_y);
  
  for (int my = wall_grid_y + 1; my < grid_size_; ++my) {
    for (int mx = 0; mx < grid_size_; ++mx) {
      if (grid_[my][mx] == '?') {
        grid_[my][mx] = 'X';
      }
    }
  }
}

void detectedLeftWall(int wall_grid_x, int wall_grid_y) {
  RCLCPP_INFO(this->get_logger(), "Detected LEFT wall at grid (%d,%d)", wall_grid_x, wall_grid_y);
  
  for (int my = 0; my < grid_size_; ++my) {
    for (int mx = 0; mx < wall_grid_x; ++mx) {
      if (grid_[my][mx] == '?') {
        grid_[my][mx] = 'X';
      }
    }
  }
}

void detectedRightWall(int wall_grid_x, int wall_grid_y) {
  RCLCPP_INFO(this->get_logger(), "Detected RIGHT wall at grid (%d,%d)", wall_grid_x, wall_grid_y);
  
  for (int my = 0; my < grid_size_; ++my) {
    for (int mx = wall_grid_x + 1; mx < grid_size_; ++mx) {
      if (grid_[my][mx] == '?') {
        grid_[my][mx] = 'X';
      }
    }
  }
}

void detectedBottomLeftCorner(int corner_grid_x, int corner_grid_y) {
  if (!corner_00_found_) {
    corner_00_found_ = true;
    corner_00_x_ = corner_grid_x;
    corner_00_y_ = corner_grid_y;
    
    double corner_relative_to_S_x_ = (corner_grid_x - center_x_);
    double corner_relative_to_S_y_ = (center_y_ - corner_grid_y);
    
    RCLCPP_INFO(this->get_logger(),
                "Found bottom-left corner at grid position (%d,%d), relative position to Start (%.1f, %.1f)",
                corner_00_x_, corner_00_y_, corner_relative_to_S_x_, corner_relative_to_S_y_);
  }
}
  
  void onTimer() {
    custom_interfaces::msg::StateEstimationMap map_msg;
    map_msg.grid = grid_;
    map_pub_->publish(map_msg);
    custom_interfaces::msg::StateEstimationPosition pos_msg;

    if (corner_00_found_) {

      int robot_world_x_ = robot_grid_x_ - corner_00_x_;
      int robot_world_y_ = corner_00_y_ - robot_grid_y_;

      pos_msg.x = robot_world_x_;
      pos_msg.y = robot_world_y_;
      pos_pub_->publish(pos_msg);
    }else{
      pos_msg.x = robot_relative_to_S_x_;
      pos_msg.y = robot_relative_to_S_y_;      
    }
    pos_pub_->publish(pos_msg);
    publishDebugInfo();
  }


  bool isValidGridPos(int x, int y) const {
    return x >= 0 && x < grid_size_ && y >= 0 && y < grid_size_;
  }

  void markPerceptionRadiusAsFloor(int center_x, int center_y, double radius) {
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

  rclcpp::Subscription<custom_interfaces::msg::RobotStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<custom_interfaces::msg::PerceptionData>::SharedPtr perception_sub_;
  rclcpp::Publisher<custom_interfaces::msg::StateEstimationPosition>::SharedPtr pos_pub_;
  rclcpp::Publisher<custom_interfaces::msg::StateEstimationMap>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_interfaces::msg::StateEstimationDebug>::SharedPtr debug_pub_;

  int grid_size_;
  int center_x_;
  int center_y_;
  int robot_grid_x_;
  int robot_grid_y_; 
  std::vector<std::string> grid_;

  int robot_relative_to_S_x_;
  int robot_relative_to_S_y_;

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