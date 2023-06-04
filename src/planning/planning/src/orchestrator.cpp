// #include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "../include/planning/global_path_planner.hpp"
#include "../include/planning/local_path_planner.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

enum EventState { acceleration, skidpad, trackdrive, autocross };

class Planning : public rclcpp::Node {
  EventState state = skidpad;
  size_t count_;
  LocalPathPlanner* local_path_planner = new LocalPathPlanner();

  rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr vl_sub_;
  rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr track_sub_;
  rclcpp::Publisher<custom_interfaces::msg::PointArray>::SharedPtr local_pub_;
  rclcpp::Publisher<custom_interfaces::msg::PointArray>::SharedPtr global_pub_;

  void vehicle_localisation_callback(const custom_interfaces::msg::Pose msg) const {
    RCLCPP_INFO(this->get_logger(), "(%f,\t%f)", msg.position.x, msg.position.y);
  }

  void track_map_callback(const custom_interfaces::msg::ConeArray msg) {
    Track* track = new Track();
    auto cone_array = msg.cone_array;

    for (auto& cone : cone_array) {
      track->addCone(cone.position.x, cone.position.y, cone.color);
      RCLCPP_INFO(this->get_logger(), "[received] (%f, \t%f)\t%s", cone.position.x, cone.position.y,
                  cone.color.c_str());
    }

    vector<Position*> path = local_path_planner->process_new_array(track);
    publish_track_points(path);
    delete (track);

    RCLCPP_INFO(this->get_logger(), "--------------------------------------");
  }

  /**
   * Publisher point by point
   */
  void publish_track_points(vector<Position*> path) const {
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

  // Track* read_from_file(std::string filename) {
  //   std::string file_prefix = rcpputils::fs::current_path().string();
  //   std::string file_package =  file_prefix + filename;
  //   Track* track = new Track();
  //   track->fill_track(filePackage);
  //   return track;
  // }

  // void write_to_file(std::string filename, vector<Position*> path) {
  //   std::string file_prefix = rcpputils::fs::current_path().string();
  //   std::string finalPath =  filePrefix + filename;
  //   ofstream finalPathFile(finalPath);
  //   for (size_t i = 0; i < path.size(); i++)
  //     finalPathFile << path[i]->getX() << " " << path[i]->getY() << "\n";
  //   finalPathFile.close();
  // }

 public:
  Planning() : Node("planning"), count_(0) {
    vl_sub_ = this->create_subscription<custom_interfaces::msg::Pose>(
        "vehicle_localization", 10, std::bind(&Planning::vehicle_localisation_callback, this, _1));

    track_sub_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "track_map", 10, std::bind(&Planning::track_map_callback, this, _1));

    local_pub_ = this->create_publisher<custom_interfaces::msg::PointArray>("planning_local", 10);
    global_pub_ = this->create_publisher<custom_interfaces::msg::PointArray>("planning_global", 10);
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
  cout << "Finished properly!" << endl;
  return 0;
}