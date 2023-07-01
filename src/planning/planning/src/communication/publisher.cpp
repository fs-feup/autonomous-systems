// #include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "../../include/planning/pathplanner.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/point_array.hpp"
#include "rclcpp/rclcpp.hpp"

// using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
/**
 * Planning Publisher Class. Overwrites node. Allows message sending to the control section
 */
class Publisher : public rclcpp::Node {
  vector<pair<float, float>> fullPath; /**<path data container */

 public:
  /**
   * Constructor. Calls and applies the pathplanner techniques to the track.
   */
  Publisher() : Node("planning_endurance"), count_(0) {
    publisher_ = this->create_publisher<custom_interfaces::msg::PointArray>("path_topic", 10);
    // timer_ = this->create_wall_timer(
    // 500ms, std::bind(&Publisher::publish_track_points, this));

    std::string filePackage = rcpputils::fs::current_path().string();
    filePackage += "/planning/planning/files/map_mock.txt";
    std::cout << filePackage;

    Track* track = new Track();
    track->fillTrack(filePackage);
    PathPlanner* pathplanner = new PathPlanner(track);
    pathplanner->middlePath();
    fullPath = pathplanner->getPath();

    publish_track_points();
  }

 private:
  /**
   * Publisher point by point
   */
  void publish_track_points() {
    auto message = custom_interfaces::msg::PointArray();
    std::cout << "Starting publisher\n";
    for (size_t i = 0; i < fullPath.size(); i++) {
      auto point = custom_interfaces::msg::Point2d();
      point.x = fullPath[i].first;
      point.y = fullPath[i].second;
      // RCLCPP_INFO(this->get_logger(), "Publishing: x = %f | y = %f", message.x, message.y);
      message.points.push_back(point);
    }
    RCLCPP_INFO(this->get_logger(), "Publishing message with size %ld", message.points.size());
    publisher_->publish(message);
  }
  /*
  void timer_callback() {
    auto message = custom_interfaces::msg::Point2d();
    message.x = 1;
    message.y = 2;
    RCLCPP_INFO(this->get_logger(), "Publishing: x = %f | y = %f", message.x, message.y);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  */
  rclcpp::Publisher<custom_interfaces::msg::PointArray>::SharedPtr publisher_;
  size_t count_;
};

/**
 * Publisher orchestrator
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
