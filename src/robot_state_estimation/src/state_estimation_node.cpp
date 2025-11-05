// Fixed State Estimation Node: incremental map building

#include <cmath>
#include <custom_interfaces/msg/perception_data.hpp>
#include <custom_interfaces/msg/robot_status.hpp>
#include <custom_interfaces/msg/state_estimation_map.hpp>
#include <custom_interfaces/msg/state_estimation_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class StateEstimationNode : public rclcpp::Node {
public:
  StateEstimationNode()
      : Node("state_estimation"),
        grid_size_(61),
        center_x_(30),
        center_y_(30),
        robot_grid_x_(30),
        robot_grid_y_(30),
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

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&StateEstimationNode::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "State Estimation node started (grid %dx%d)", grid_size_,
                grid_size_);
  }

private:

//done??
  void onStatus(const custom_interfaces::msg::RobotStatus::SharedPtr msg) {
    // Update robot's world position based on velocity
    rclcpp::Time now = this->now();
    double dt = (now - last_update_time_).seconds();
    if (dt < 0.0) dt = 0.0;
    last_update_time_ = now;

    double vx = msg->velocity.x;
    double vy = msg->velocity.y;

    // Only update if velocity is significant
    if (std::abs(vx) > 0.01 || std::abs(vy) > 0.01) {

      // calculate distance moved
      double dist_x = vx * dt;
      double dist_y = vy * dt;

      // Update robot grid position 
      if (isValidGridPos(robot_grid_x_ + static_cast<int>(std::round(dist_x)), robot_grid_y_ + static_cast<int>(std::round(dist_y)))) {
        // Update robot grid position 
        robot_grid_x_ += static_cast<int>(std::round(dist_x));
        robot_grid_y_ += static_cast<int>(std::round(dist_y));

        if(robot_world_x_!=0 && robot_world_y_ != 0){
          robot_world_x_ += static_cast<int>(std::round(dist_x));
          robot_world_y_ += static_cast<int>(std::round(dist_y));
        }

        // Update relative position to S
        robot_relative_to_S_x_ += static_cast<int>(std::round(dist_x));
        robot_relative_to_S_y_ += static_cast<int>(std::round(dist_y));

        // Mark current robot position as floor ('0'), preserve 'S' if at start
        if (grid_[robot_grid_y_][robot_grid_x_] == '?'){
          grid_[robot_grid_y_][robot_grid_x_] = '0';
        }

      }

    }
  }

//done?
  void onPerception(const custom_interfaces::msg::PerceptionData::SharedPtr msg) {

    double perception_radius = msg->perception_radius;

    // First pass -> mark all positions within perception radius as floor ('0') unless they're already marked with an object
    markPerceptionRadiusAsFloor(robot_grid_x_, robot_grid_y_, perception_radius); //correto

    // Second pass: place all detected objects on the grid
    for (const auto& obj : msg->objects) {
      const std::string& type = obj.object_type;
      const std::string& id = obj.object_id;
      double rel_x = obj.relative_position.x;
      double rel_y = obj.relative_position.y;

      // Calculate grid position relative to current robot position and relative position to S
      int obj_grid_x = robot_grid_x_ + static_cast<int>(std::round(rel_x));
      int obj_grid_y = robot_grid_y_ + static_cast<int>(std::round(rel_y));

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
  }

//done?
  void detectBottomLeftCorner() {
    // Scan the grid to find a bottom-left corner pattern:
    // A corner is where we have walls forming an L-shape
    // Pattern: wall at (x,y), wall at (x+1,y), wall at (x,y+1)

    for (int y = 0; y < grid_size_ - 1; ++y) {
      for (int x = 0; x < grid_size_ - 1; ++x) {
        if (grid_[y][x] != '9') continue;  // current cell must be a wall
  
        bool wall_right = (grid_[y][x + 1] == '9');
        bool wall_above = (grid_[y + 1][x] == '9');
  
        // Check the simple L-shape pattern
        if (wall_right && wall_above) {
          corner_00_found_ = true;
          corner_00_x_ = x;
          corner_00_y_ = y;
  
          double corner_relative_to_S_x_ = (x - center_x_);
          double corner_relative_to_S_y_ = (y - center_y_);
  
          RCLCPP_INFO(this->get_logger(),
                      "Found bottom-left corner at grid position (%d,%d), relative position to Start (%.1f, %.1f)",
                      corner_00_x_, corner_00_y_, corner_relative_to_S_x_, corner_relative_to_S_y_);
  
          // mark out-of-bounds to the left and below, same as before
          for (int mx = 0; mx < x; ++mx) {
            for (int my = 0; my < grid_size_; ++my) {
              if (grid_[my][mx] == '?') grid_[my][mx] = '.';
            }
          }
          for (int my = 0; my < y; ++my) {
            for (int mx = 0; mx < grid_size_; ++mx) {
              if (grid_[my][mx] == '?') grid_[my][mx] = '.';
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

  // Grid representation: 61x61, center at (30, 30)
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