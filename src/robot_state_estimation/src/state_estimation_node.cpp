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
        robot_world_x_(0.0),
        robot_world_y_(0.0),
        corner_00_x_(0),
        corner_00_y_(0),
        corner_00_found_(false),
        found_wall_negative_x_(false),
        found_wall_negative_y_(false),
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
//funcoes callbacks ->>>

//isto nao pode tar correto
  void onStatus(const custom_interfaces::msg::RobotStatus::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "onStatus called");
    // Update robot's world position based on velocity
    rclcpp::Time now = this->now();
    double dt = (now - last_update_time_).seconds();
    if (dt < 0.0) dt = 0.0;
    last_update_time_ = now;

    double vx = msg->velocity.x;
    double vy = msg->velocity.y;

    // Only update if velocity is significant
    if (std::abs(vx) > 0.01 || std::abs(vy) > 0.01) {
      // Store previous grid position
      int prev_grid_x = robot_grid_x_;
      int prev_grid_y = robot_grid_y_;

      // Update world position
      robot_world_x_ += vx * dt;
      robot_world_y_ += vy * dt;

      // Update grid position (relative to center)
      robot_grid_x_ = center_x_ + static_cast<int>(std::round(robot_world_x_));
      robot_grid_y_ = center_y_ + static_cast<int>(std::round(robot_world_y_));

      // Mark all positions between previous and current as floor ('0')
      markPathAsFloor(prev_grid_x, prev_grid_y, robot_grid_x_, robot_grid_y_);

      // Mark current robot position as floor ('0'), preserve 'S' if at start
      if (isValidGridPos(robot_grid_x_, robot_grid_y_)) {
        if (grid_[robot_grid_y_][robot_grid_x_] == '?') {
          grid_[robot_grid_y_][robot_grid_x_] = '0';
        }
      }
    }
  }

  void onPerception(const custom_interfaces::msg::PerceptionData::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "onPerception called");

    double perception_radius = msg->perception_radius;

    // First pass -> mark all positions within perception radius as floor ('0') unless they're already marked with an object
    markPerceptionRadiusAsFloor(robot_grid_x_, robot_grid_y_, perception_radius); //correto

    // Second pass: place all detected objects on the grid
    for (const auto& obj : msg->objects) {
      const std::string& type = obj.object_type;
      const std::string& id = obj.object_id;
      double rel_x = obj.relative_position.x;
      double rel_y = obj.relative_position.y;

      // Calculate grid position relative to current robot position
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

  void detectBottomLeftCorner() {
    // Scan the grid to find a bottom-left corner pattern:
    // A corner is where we have walls forming an L-shape
    // Pattern: wall at (x,y), wall at (x-1,y), wall at (x,y-1)
    // And no walls at (x+1,y) and (x,y+1) - those should be inside the map

    for (int y = 1; y < grid_size_ - 1; ++y) {
      for (int x = 1; x < grid_size_ - 1; ++x) {
        // Check if current cell is a wall
        if (grid_[y][x] != '9') continue;

        // Check if there's a wall to the left (x-1, y)
        bool wall_left = (grid_[y][x - 1] == '9');

        // Check if there's a wall below (x, y-1)
        bool wall_below = (grid_[y - 1][x] == '9');

        // Check if there's out-of-bounds to the left and below
        bool oob_left = (x > 0 && grid_[y][x - 1] == '.');
        bool oob_below = (y > 0 && grid_[y - 1][x] == '.');

        // Bottom-left corner: wall here, and either walls or out-of-bounds on left and below
        if ((wall_left || oob_left) && (wall_below || oob_below)) {
          // Additional check: make sure there's empty space or unknown to the right and above
          // (inside the map)
          bool space_right =
              (x < grid_size_ - 1 && grid_[y][x + 1] != '9' && grid_[y][x + 1] != '.');
          bool space_above =
              (y < grid_size_ - 1 && grid_[y + 1][x] != '9' && grid_[y + 1][x] != '.');

          if (space_right || space_above) {
            // Found the corner!
            corner_00_found_ = true;
            corner_00_x_ = x;
            corner_00_y_ = y;

            // Calculate the offset: this corner is at (0,0) in absolute coordinates
            // Robot's absolute position = robot_world_position + offset
            // The corner's world position relative to robot is: (x - 30, y - 30)
            double corner_world_x = (x - robot_grid_x_);
            double corner_world_y = (y - robot_grid_y_);

            // Since corner is at absolute (0,0), robot's absolute position is:
            // robot_abs = -corner_relative_position
            // But we're already tracking robot_world_x/y, so we need to adjust them
            // Actually, robot's absolute position is: robot_world - corner_world
            // No wait: if corner is at relative position (corner_world_x, corner_world_y)
            // and corner is at absolute (0,0), then robot is at absolute (-corner_world_x,
            // -corner_world_y) We need to remember this offset

            RCLCPP_INFO(this->get_logger(),
                        "Found bottom-left corner at grid(%d,%d), which is at relative position "
                        "(%.1f,%.1f) from robot",
                        corner_00_x_, corner_00_y_, corner_world_x, corner_world_y);
            RCLCPP_INFO(this->get_logger(), "Robot absolute position is (%.2f, %.2f)",
                        robot_world_x_ - corner_world_x, robot_world_y_ - corner_world_y);

            // Mark all cells to the left and below as out of bounds
            for (int mx = 0; mx < x; ++mx) {
              for (int my = 0; my < grid_size_; ++my) {
                if (grid_[my][mx] == '?') {
                  grid_[my][mx] = '.';
                }
              }
            }
            for (int my = 0; my < y; ++my) {
              for (int mx = 0; mx < grid_size_; ++mx) {
                if (grid_[my][mx] == '?') {
                  grid_[my][mx] = '.';
                }
              }
            }

            return;  // Found it, stop searching
          }
        }
      }
    }
  }

  void onTimer() {
    // Always publish the map
    custom_interfaces::msg::StateEstimationMap map_msg;
    map_msg.grid = grid_;
    map_pub_->publish(map_msg);

    // Only publish position if bottom-left corner (0,0) has been found
    if (corner_00_found_) {
      // Calculate robot's absolute position
      // The corner is at grid position (corner_00_x_, corner_00_y_)
      // which is at relative position (corner_00_x_ - 30, corner_00_y_ - 30) from robot
      // Since corner is at absolute (0,0), robot's absolute position is:
      double corner_rel_x = corner_00_x_ - robot_grid_x_;
      double corner_rel_y = corner_00_y_ - robot_grid_y_;

      custom_interfaces::msg::StateEstimationPosition pos_msg;
      pos_msg.x = robot_world_x_ - corner_rel_x;
      pos_msg.y = robot_world_y_ - corner_rel_y;
      pos_pub_->publish(pos_msg);
    }
  }

  // --- Helpers ---
  bool isValidGridPos(int x, int y) const {
    return x >= 0 && x < grid_size_ && y >= 0 && y < grid_size_;
  }

  //isto ta correto -> marcar todas as celulas dentro do percpetion radius como 0 se nao conhecidas
  void markPerceptionRadiusAsFloor(int center_x, int center_y, double radius) {
    int radius_int = static_cast<int>(std::ceil(radius));
    for (int dy = -radius_int; dy <= radius_int; ++dy) {
      for (int dx = -radius_int; dx <= radius_int; ++dx) {
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist <= radius) {
          int x = center_x + dx;
          int y = center_y + dy;
          if (isValidGridPos(x, y)) {
            // Only mark unknown cells as floor, preserve 'S' at start
            if (grid_[y][x] == '?') {
              grid_[y][x] = '0';
            }
          }
        }
      }
    }
  }

  void markPathAsFloor(int x1, int y1, int x2, int y2) {
    // Mark all cells on the path from (x1,y1) to (x2,y2) as floor
    // Use Bresenham-like line algorithm
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    int x = x1;
    int y = y1;

    while (true) {
      if (isValidGridPos(x, y)) {
        // Don't overwrite 'S' at start position
        if (grid_[y][x] == '?') {
          grid_[y][x] = '0';
        }
      }

      if (x == x2 && y == y2) break;

      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x += sx;
      }
      if (e2 < dx) {
        err += dx;
        y += sy;
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
    return '?';  // Unknown type
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

  // Robot position in world coordinates (relative to where it started)
  double robot_world_x_;
  double robot_world_y_;

  // Corner (0,0) detection
  int corner_00_x_;
  int corner_00_y_;
  bool corner_00_found_;
  bool found_wall_negative_x_;  // Not used anymore but kept for compatibility
  bool found_wall_negative_y_;  // Not used anymore but kept for compatibility

  rclcpp::Time last_update_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimationNode>());
  rclcpp::shutdown();
  return 0;
}