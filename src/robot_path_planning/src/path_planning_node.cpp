#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/msg/state_estimation_map.hpp>
#include <custom_interfaces/msg/state_estimation_debug.hpp>
#include <custom_interfaces/msg/robot_path_point.hpp>
#include <custom_interfaces/msg/path.hpp>
#include <vector>
#include <string>
#include <queue>
#include <cmath>
#include <algorithm>
#include <set>

// will remove this if i try to change state estimation map to give a clean map without 'X'
#include <queue>
#include <utility>
const int MAP_HEIGHT = 101;
const int MAP_WIDTH = 101;
const std::string KNOWN_CHARS = "0S12345RABCDE";


struct Point {
  int x, y;
  Point(int x_ = 0, int y_ = 0) : x(x_), y(y_) {}
  bool operator==(const Point& other) const {
    return x == other.x && y == other.y;
  }
  bool operator<(const Point& other) const {
    if (x != other.x) return x < other.x;
    return y < other.y;
  }
};

class PathPlanningNode : public rclcpp::Node {
public:
  PathPlanningNode() : Node("path_planning_node"), 
                       map_received_(false),
                       position_received_(false),
                       exploration_complete_(false),
                       delivery_complete_(false),
                       robot_world_x_(0),
                       robot_world_y_(0) {
    
    map_sub_ = this->create_subscription<custom_interfaces::msg::StateEstimationMap>(
        "state_estimation/map", 10,
        std::bind(&PathPlanningNode::mapCallback, this, std::placeholders::_1));
    
    path_pub_ = this->create_publisher<custom_interfaces::msg::Path>(
        "planning/path", 10);

    position_sub_ = this->create_subscription<custom_interfaces::msg::StateEstimationDebug>(
        "state_estimation/debug", 10,  
        std::bind(&PathPlanningNode::positionCallback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PathPlanningNode::planningLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Path Planning node initialized");
  }

private:
  void mapCallback(const custom_interfaces::msg::StateEstimationMap::SharedPtr msg) {
    map_ = msg->grid;
    map_received_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "Map received: %dx%d", 
                 static_cast<int>(map_.size()), 
                 map_.empty() ? 0 : static_cast<int>(map_[0].size()));
  }
  
  void positionCallback(const custom_interfaces::msg::StateEstimationDebug::SharedPtr msg) {
    robot_pos_.x = msg->robot_grid_x;
    robot_pos_.y = msg->robot_grid_y;
    position_received_ = true;
  }
  
  void planningLoop() {
    //acrescentar logica para dar pick ou drop, se fizer sentido, durante 1 fase
    if (!map_received_ || !position_received_) {
      static int log_counter = 0;
      if (log_counter++ % 10 == 0) {  //  vai dar log a cada 5 segundos (500ms * 10)
        RCLCPP_WARN(this->get_logger(), 
                    "Waiting for data - Map: %s, Position: %s",
                    map_received_ ? "YES" : "NO",
                    position_received_ ? "YES" : "NO");
      }
      return;
    }
    
    // 1 fase - explorar o mapa
    if (!exploration_complete_) {
      if (isMapFullyExplored(map_)) {
        exploration_complete_ = true;
        RCLCPP_INFO(this->get_logger(), "Map exploration complete!");
      } else {
        exploremap();
      }
    }
    // 2 fase - colocar boxes nas shelves
    else if (!delivery_complete_) {
      deliverBoxes();
    }
  }
  
  bool isMapFullyExplored(const std::vector<std::string>& map_) {//vai mudar quando mudar state estimation
    if (map_.empty() || map_[0].empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Map is empty");
        return false;
    }

    int start_row = robot_pos_.y; 
    int start_col = robot_pos_.x;  

    RCLCPP_DEBUG(this->get_logger(), "Checking exploration from grid [%d, %d]", start_row, start_col);

    // BFS from start
    std::queue<std::pair<int, int>> q;
    q.push({start_row, start_col});

    std::vector<std::vector<bool>> visited(MAP_HEIGHT, std::vector<bool>(MAP_WIDTH, false));
    visited[start_row][start_col] = true;

    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};

    while (!q.empty()) {
        std::pair<int, int> current = q.front();
        q.pop();
        int r = current.first;
        int c = current.second;

        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i];
            int nc = c + dc[i];

            if (nr >= 0 && nr < MAP_HEIGHT && nc >= 0 && nc < MAP_WIDTH) {
                char neighbor_char = map_[nr][nc];

                if (neighbor_char == '?') {
                    RCLCPP_DEBUG(this->get_logger(), 
                                "Found unknown cell '?' at grid [%d, %d] - exploration not complete",
                                nr, nc);
                    return false;
                }

                if (KNOWN_CHARS.find(neighbor_char) != std::string::npos && !visited[nr][nc]) {
                    visited[nr][nc] = true;
                    q.push({nr, nc});
                }
            }
        }
    }

    return true;
  }
  
  void exploremap() {
    Point frontier = findNearestFrontier();
    
    if (frontier.x == -1) {
      RCLCPP_WARN(this->get_logger(), "No frontiers found - marking exploration complete");
      exploration_complete_ = true;
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "Found frontier at grid [%d, %d], robot at grid [%d, %d]",
                frontier.x, frontier.y, robot_pos_.x, robot_pos_.y);
    
    std::vector<Point> path = planPath(robot_pos_, frontier);
    
    if (path.empty()) {
      RCLCPP_WARN(this->get_logger(), 
                  "Could not plan path to frontier at [%d, %d]",
                  frontier.x, frontier.y);
    } else {
      // remover o primeiro waypoint se for igual à posicao do robot
      if (!path.empty() && path[0] == robot_pos_) {
        path.erase(path.begin());
      }
      
      RCLCPP_INFO(this->get_logger(), 
                  "Planned path to frontier with %zu waypoints",
                  path.size());
      publishPath(path);
    }
  }
  
  Point findNearestFrontier() {
    if (map_.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Map empty in findNearestFrontier");
        return Point(-1, -1);
    }
    int map_height = map_.size();
    int map_width = map_[0].size();
    std::vector<Point> frontiers;
    

    for (int grid_y = 0; grid_y < map_height; ++grid_y) {
        for (int grid_x = 0; grid_x < map_width; ++grid_x) {
            char cell = map_[grid_y][grid_x];
            
            // Check if this is a known floor cell
            if (cell == '0' || cell == 'R' || cell == 'S' || cell == 'A'|| cell == '9' || cell == '1' || cell == 'x' ) {
                // Check if adjacent to unknown AND has a clear path (not blocked by walls)
                if (isAdjacentToUnknownGrid(grid_x, grid_y) && hasAccessibleUnknown(grid_x, grid_y)) {
                    frontiers.push_back(Point(grid_x, grid_y));
                }
            }
        }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Found %zu frontier cells", frontiers.size());
    
    if (frontiers.empty()) {
        return Point(-1, -1);
    }
    
    Point nearest = frontiers[0];
    double min_dist = distance(robot_pos_, nearest);
    
    for (const auto& f : frontiers) {
        double dist = distance(robot_pos_, f);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = f;
        }
    }
    
    return nearest;
}

// Helper function to check if a frontier has accessible unknown cells
bool hasAccessibleUnknown(int grid_x, int grid_y) {
    int map_height = map_.size();
    int map_width = map_[0].size();
    
    int dx[] = {0, 1, 0, -1};
    int dy[] = {-1, 0, 1, 0};
    
    for (int i = 0; i < 4; ++i) {
        int nx = grid_x + dx[i];
        int ny = grid_y + dy[i];
        
        if (nx >= 0 && nx < map_width && ny >= 0 && ny < map_height) {
            if (map_[ny][nx] == '?') {
                return true;
            }
        }
    }
    
    return false;
}
  bool isAdjacentToUnknownGrid(int grid_x, int grid_y) {
    int dx[] = {0, 1, 0, -1};
    int dy[] = {-1, 0, 1, 0};
    
    for (int i = 0; i < 4; ++i) {
      int nx = grid_x + dx[i];
      int ny = grid_y + dy[i];
      
      if (ny >= 0 && ny < map_.size() && nx >= 0 && nx < map_[0].size()) {
        if (map_[ny][nx] == '?') {
          return true;
        }
      }
    }
    return false;
  }
  
  void deliverBoxes() { // was not able to test yet
    std::vector<Point> boxes[5]; 
    std::vector<Point> shelves[5];
    
    if (map_.empty()) return;
    
    int map_height = map_.size();
    int map_width = map_[0].size();
    
    for (int grid_y = 0; grid_y < map_height; ++grid_y) {
      for (int grid_x = 0; grid_x < map_width; ++grid_x) {
        char cell = map_[grid_y][grid_x];
        
        if (cell >= '1' && cell <= '5') {
          boxes[cell - '1'].push_back(Point(grid_x, grid_y));
        } else if (cell >= 'A' && cell <= 'E') {
          shelves[cell - 'A'].push_back(Point(grid_x, grid_y));
        }
      }
    }
    
    for (int i = 0; i < 5; ++i) {
      if (!boxes[i].empty() || !shelves[i].empty()) {
        RCLCPP_DEBUG(this->get_logger(), 
                    "Type %d: %zu boxes, %zu shelves",
                    i + 1, boxes[i].size(), shelves[i].size());
      }
    }
    
    Point best_box(-1, -1);
    Point best_shelf(-1, -1);
    double best_total_dist = std::numeric_limits<double>::max();
    
    for (int i = 0; i < 5; ++i) {
      if (!boxes[i].empty() && !shelves[i].empty()) {
        for (const auto& box : boxes[i]) {
          for (const auto& shelf : shelves[i]) {
            double dist_to_box = distance(robot_pos_, box);
            double dist_box_to_shelf = distance(box, shelf);
            double total_dist = dist_to_box + dist_box_to_shelf;
            
            if (total_dist < best_total_dist) {
              best_total_dist = total_dist;
              best_box = box;
              best_shelf = shelf;
            }
          }
        }
      }
    }
    
    if (best_box.x == -1) {
      delivery_complete_ = true;
      RCLCPP_INFO(this->get_logger(), "All boxes delivered!");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "Planning delivery: box at [%d, %d] to shelf at [%d, %d]",
                best_box.x, best_box.y, best_shelf.x, best_shelf.y);
    
    std::vector<Point> path_to_box = planPath(robot_pos_, best_box);
    std::vector<Point> path_to_shelf = planPath(best_box, best_shelf);
    
    std::vector<Point> full_path = path_to_box;
    full_path.insert(full_path.end(), path_to_shelf.begin(), path_to_shelf.end());
    
    if (!full_path.empty()) {
      publishPath(full_path);
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not plan delivery path");
    }
  }
  
  std::vector<Point> planPath(const Point& start, const Point& goal) {
    if (map_.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "Map empty in planPath");
      return {};
    }
    
    RCLCPP_DEBUG(this->get_logger(), 
                "Planning path from [%d, %d] to [%d, %d]",
                start.x, start.y, goal.x, goal.y);
    
    struct Node {
      Point pos;
      double g, h;
      Point parent;
      
      double f() const { return g + h; }
      
      bool operator>(const Node& other) const {
        return f() > other.f();
      }
    };
    
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::set<Point> closed;
    std::map<Point, Point> came_from;
    std::map<Point, double> g_score;
    
    Node start_node;
    start_node.pos = start;
    start_node.g = 0;
    start_node.h = distance(start, goal);
    start_node.parent = Point(-1, -1);
    
    open.push(start_node);
    g_score[start] = 0;
    
    int dx[] = {0, 1, 0, -1};
    int dy[] = {-1, 0, 1, 0};
    
    while (!open.empty()) {
      Node current = open.top();
      open.pop();
      
      if (current.pos == goal) {
        std::vector<Point> path;
        Point p = goal;
        while (!(p.x == -1 && p.y == -1)) {
          path.push_back(p);
          if (came_from.find(p) != came_from.end()) {
            p = came_from[p];
          } else {
            break;
          }
        }
        std::reverse(path.begin(), path.end());
        
        RCLCPP_DEBUG(this->get_logger(), "Found path with %zu waypoints", path.size());
        return path;
      }
      
      if (closed.count(current.pos)) continue;
      closed.insert(current.pos);
      
      for (int i = 0; i < 4; ++i) {
        int nx = current.pos.x + dx[i];
        int ny = current.pos.y + dy[i];
        Point neighbor(nx, ny);
        
        if (!isWalkableGrid(nx, ny)) continue;
        if (closed.count(neighbor)) continue;
        
        double tentative_g = current.g + 1.0;
        
        if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
          came_from[neighbor] = current.pos;
          g_score[neighbor] = tentative_g;
          
          Node neighbor_node;
          neighbor_node.pos = neighbor;
          neighbor_node.g = tentative_g;
          neighbor_node.h = distance(neighbor, goal);
          neighbor_node.parent = current.pos;
          
          open.push(neighbor_node);
        }
      }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "No path found");
    return {};
  }
  
  bool isWalkableGrid(int grid_x, int grid_y) {
    if (map_.empty()) return false;
    
    int map_height = map_.size();
    int map_width = map_[0].size();
    
    if (grid_y < 0 || grid_y >= map_height || grid_x < 0 || grid_x >= map_width) {
      return false;
    }
    
    char cell = map_[grid_y][grid_x];
    return cell == '0' || cell == 'R' || cell == 'S' || 
           (cell >= '1' && cell <= '5') || (cell >= 'A' && cell <= 'E');
  }
  
  double distance(const Point& a, const Point& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }
  
  
  void publishPath(const std::vector<Point>& path) {
    custom_interfaces::msg::Path path_msg;
    
    for (const auto& p : path) {
      custom_interfaces::msg::RobotPathPoint point;
      point.x = static_cast<int32_t>(p.x);
      point.y = static_cast<int32_t>(p.y);
      path_msg.points.push_back(point);
    }
    
    path_pub_->publish(path_msg);
    
    RCLCPP_INFO(this->get_logger(), "Published path with %zu points", path.size());
  }
  
  rclcpp::Subscription<custom_interfaces::msg::StateEstimationMap>::SharedPtr map_sub_;
  rclcpp::Subscription<custom_interfaces::msg::StateEstimationDebug>::SharedPtr position_sub_;
  rclcpp::Publisher<custom_interfaces::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::vector<std::string> map_;
  Point robot_pos_; 
  int robot_world_x_;  
  int robot_world_y_;
  bool map_received_;
  bool position_received_;
  bool exploration_complete_;
  bool delivery_complete_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanningNode>());
  rclcpp::shutdown();
  return 0;
}