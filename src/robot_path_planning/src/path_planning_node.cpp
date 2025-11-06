// Simple frontier-based planning with A* over occupancy grid

#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/msg/state_estimation_position.hpp>
#include <custom_interfaces/msg/state_estimation_map.hpp>
#include <custom_interfaces/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <queue>
#include <optional>
#include <limits>
#include <cmath>

class PathPlanningNode : public rclcpp::Node {
public:
  PathPlanningNode() : Node("path_planning") {
    using std::placeholders::_1;
    pos_sub_ = this->create_subscription<custom_interfaces::msg::StateEstimationPosition>(
      "state_estimation/position", 10, std::bind(&PathPlanningNode::onPosition, this, _1));
    map_sub_ = this->create_subscription<custom_interfaces::msg::StateEstimationMap>(
      "state_estimation/map", 10, std::bind(&PathPlanningNode::onMap, this, _1));
    path_pub_ = this->create_publisher<custom_interfaces::msg::Path>("planning/path", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PathPlanningNode::onTimer, this));
  }

private:
  void onPosition(const custom_interfaces::msg::StateEstimationPosition::SharedPtr msg) {
    rx_ = msg->x; ry_ = msg->y; have_pos_ = true;
  }
  void onMap(const custom_interfaces::msg::StateEstimationMap::SharedPtr msg) {
    width_ = msg->width; height_ = msg->height; data_ = msg->data; have_map_ = true;
  }
  void onTimer() {
    if (!have_pos_ || !have_map_ || data_.empty() || width_ <= 0 || height_ <= 0) return;
    // Stop if no unknown cells remain (map fully explored)
    bool has_unknown = false; for (auto v : data_) { if (v == -1) { has_unknown = true; break; } }
    if (!has_unknown) return;
    auto start = worldToIndex(rx_, ry_);
    if (!start) return;
    int goal = pickFrontier(*start);
    if (goal < 0) return;
    auto cells = astar(*start, goal);
    if (cells.empty()) return;
    custom_interfaces::msg::Path path;
    path.waypoints.reserve(cells.size());
    for (int idx : cells) path.waypoints.push_back(cellCenter(idx));
    path_pub_->publish(path);
  }

  // helpers
  bool isFree(int idx) const { return 0 <= idx && idx < static_cast<int>(data_.size()) && data_[idx] == 0; }
  bool isUnknown(int idx) const { return 0 <= idx && idx < static_cast<int>(data_.size()) && data_[idx] == -1; }
  std::optional<int> worldToIndex(double wx, double wy) const {
    int ix = static_cast<int>(std::lround(wx));
    int iy = static_cast<int>(std::lround(wy));
    if (0 <= ix && ix < width_ && 0 <= iy && iy < height_) return iy * width_ + ix;
    return std::nullopt;
  }
  geometry_msgs::msg::Point cellCenter(int idx) const {
    geometry_msgs::msg::Point p; p.x = static_cast<double>(idx % width_); p.y = static_cast<double>(idx / width_); p.z = 0.0; return p;
  }
  void neighbors4(int idx, std::vector<int> & out) const {
    out.clear();
    int x = idx % width_, y = idx / width_;
    if (x > 0) out.push_back(idx - 1);
    if (x + 1 < width_) out.push_back(idx + 1);
    if (y > 0) out.push_back(idx - width_);
    if (y + 1 < height_) out.push_back(idx + width_);
  }
  int pickFrontier(int start) const {
    int sx = start % width_, sy = start / width_;
    int best = -1; int best_cost = std::numeric_limits<int>::max();
    std::vector<int> neigh;
    for (int i = 0; i < static_cast<int>(data_.size()); ++i) {
      if (!isFree(i)) continue;
      neighbors4(i, neigh);
      bool touches_unknown = false; for (int n : neigh) if (isUnknown(n)) { touches_unknown = true; break; }
      if (!touches_unknown) continue;
      int x = i % width_, y = i / width_;
      int cost = std::abs(x - sx) + std::abs(y - sy);
      if (cost < best_cost) { best_cost = cost; best = i; }
    }
    return best;
  }
  std::vector<int> astar(int start, int goal) const {
    struct Q { int f; int idx; };
    auto cmp = [](const Q & a, const Q & b){ return a.f > b.f; };
    std::priority_queue<Q, std::vector<Q>, decltype(cmp)> open(cmp);
    std::vector<int> came(width_ * height_, -2);
    std::vector<int> g(width_ * height_, std::numeric_limits<int>::max());
    auto h = [&](int i){ int x=i%width_, y=i/width_, gx=goal%width_, gy=goal/width_; return std::abs(x-gx)+std::abs(y-gy); };
    g[start] = 0; came[start] = -1; open.push({h(start), start});
    std::vector<int> neigh;
    while(!open.empty()){
      int cur = open.top().idx; open.pop();
      if (cur == goal) break;
      neighbors4(cur, neigh);
      for (int n : neigh) {
        if (!isFree(n)) continue;
        int tentative = g[cur] + 1;
        if (tentative < g[n]) { g[n] = tentative; came[n] = cur; open.push({tentative + h(n), n}); }
      }
    }
    if (came[goal] == -2) return {};
    std::vector<int> path; int cur = goal; while (cur != -1) { path.push_back(cur); cur = came[cur]; }
    std::reverse(path.begin(), path.end());
    return path;
  }

  rclcpp::Subscription<custom_interfaces::msg::StateEstimationPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<custom_interfaces::msg::StateEstimationMap>::SharedPtr map_sub_;
  rclcpp::Publisher<custom_interfaces::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool have_pos_{false}, have_map_{false};
  double rx_{0.0}, ry_{0.0};
  int width_{0}, height_{0};
  std::vector<int8_t> data_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanningNode>());
  rclcpp::shutdown();
  return 0;
}
