#include <functional>
#include <memory>
#include <string>

#include "../include/planning/global_path_planner.hpp"
#include "../include/planning/local_path_planner.hpp"
#include "../include/utils/files.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"

using std::placeholders::_1;

enum EventState { acceleration, skidpad, trackdrive, autocross };

class Planning : public rclcpp::Node {
  EventState state = skidpad;
  LocalPathPlanner* local_path_planner = new LocalPathPlanner();

  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr vl_sub_;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr track_sub_;
  rclcpp::Publisher<custom_interfaces::msg::PointArray>::SharedPtr local_pub_;
  rclcpp::Publisher<custom_interfaces::msg::PointArray>::SharedPtr global_pub_;
  float initialOrientation_;

  void vehicle_localisation_callback(const custom_interfaces::msg::Pose msg) {
    RCLCPP_INFO(this->get_logger(), "Vehicle Localisation: (%f, %f, %fdeg)",
      msg.position.x, msg.position.y, msg.theta);
    if (initialOrientation_ == -1) {
      initialOrientation_ = msg.theta;
      local_path_planner->setOrientation(msg.theta);
      RCLCPP_INFO(this->get_logger(), "Orientation set to %f degrees.", initialOrientation_);
    }
  }

  void track_map_callback(const custom_interfaces::msg::ConeArray msg) {
    Track* track = new Track();
    auto cone_array = msg.cone_array;

    for (auto& cone : cone_array) {
      track->addCone(cone.position.x, cone.position.y, cone.color);
      RCLCPP_INFO(this->get_logger(), "[received] (%f, \t%f)\t%s", cone.position.x, cone.position.y,
                  cone.color.c_str());
    }

    //std::vector<Position*> path = local_path_planner->processNewArray(track);
    //publish_track_points(path);
    delete (track);

    RCLCPP_INFO(this->get_logger(), "--------------------------------------");
  }

  /**
   * Publisher point by point
   */
  void publish_track_points(std::vector<Position*> path) const {
    auto message = custom_interfaces::msg::PointArray();
    for (auto const& element : path) {
      auto point = custom_interfaces::msg::Point2d();
      point.x = element->getX();
      point.y = element->getY();
      message.points.push_back(point);
    }
    local_pub_->publish(message);

    RCLCPP_INFO(this->get_logger(), "[published] %ld points", message.points.size());
  }

 public:
  Planning() : Node("planning"),  initialOrientation_(-1) {
    vl_sub_ = this->create_subscription<custom_interfaces::msg::Pose>(
        "vehicle_localization", 10, std::bind(&Planning::vehicle_localisation_callback, this, _1));

    track_sub_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "track_map", 10, std::bind(&Planning::track_map_callback, this, _1));

    local_pub_ = this->create_publisher<custom_interfaces::msg::PointArray>("planning_local", 10);
    global_pub_ = this->create_publisher<custom_interfaces::msg::PointArray>("planning_global", 10);

    // test only
    std::cout << "Testing planning from file.\n";
    Track* track = read_track_file("hairpins.txt");
    std::vector<Position*> fullPath = local_path_planner->processNewArray(track);
    write_path_file("finalPath.txt", fullPath);
    std::cout << "Writing test planning to file with size " << fullPath.size() << "\n";
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto planning = std::make_shared<Planning>();
  try {
    rclcpp::spin(planning);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(planning->get_logger(), "%s", e.what());
    throw std::runtime_error("Planning runtime error!");
  }
  rclcpp::shutdown();
  std::cout << "Finished properly!\n";
  return 0;
}