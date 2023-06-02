// #include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "../include/planning/global_path_planner.hpp"
#include "../include/planning/local_path_planner.hpp"

#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

enum EventState{
  acceleration,
  skidpad,
  trackdrive,
  autocross
};

class Planning : public rclcpp::Node {
  EventState state = skidpad;
  vector<Position*> fullPath;
  LocalPathPlanner* local_path_planner = new LocalPathPlanner();

 public:
    Planning()
    : Node("planning"), count_(0) {
      vl_sub_ = this->create_subscription<custom_interfaces::msg::Pose>(
      "vehicle_localization", 10, std::bind(&Planning::vehicle_localisation_callback, this, _1));

      track_sub_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "track_map", 10, std::bind(&Planning::track_map_callback, this, _1));

      local_pub_ = this->create_publisher<custom_interfaces::msg::PointArray>
      ("planning_local", 10);
      global_pub_ = this->create_publisher<custom_interfaces::msg::PointArray>
      ("planning_global", 10);

      // =========== Tmp read loc_map from file ==============
      // std::string filePrefix = rcpputils::fs::current_path().string();
      // std::string filePackage =  filePrefix + "/planning/planning/files/skidpad.txt";
      // Track* track = new Track();
      // track->fillTrack(filePackage);
      // =====================================================

      // fullPath = local_path_planner->processNewArray(track); // test only

      // ============= Tmp write path to file ================
      // std::string finalPath =  filePrefix + "/planning/planning/files/finalPath.txt";
      // ofstream finalPathFile(finalPath);
      // for (size_t i = 0; i <fullPath.size(); i++)
      //   finalPathFile << fullPath[i]->getX() << " " << fullPath[i]->getY() << "\n";
      // finalPathFile.close();
      // =====================================================

      // GlobalPathPlanner* globalpathplanner = new GlobalPathPlanner(track);
      // globalpathplanner->middlePath();
      // globalpathplanner->writeGlobalPath(filePrefix);
      // fullPath = globalpathplanner->getPath();

      // publish_track_points();
    }

 private:
    void vehicle_localisation_callback(const custom_interfaces::msg::Pose msg) const {
      RCLCPP_INFO(this->get_logger(), "(%f, %f)", msg.position.x, msg.position.y);
    }

    void track_map_callback(const custom_interfaces::msg::ConeArray msg) {
      Track* track = new Track();
      auto cone_array = msg.cone_array;

      for (auto& cone : cone_array) {
        track->addCone(cone.position.x, cone.position.y, cone.color);
        RCLCPP_INFO(this->get_logger(), "(%f, %f)\t%s", cone.position.x, cone.position.y, cone.color.c_str());
      }
      RCLCPP_INFO(this->get_logger(), "--------------------------------------");

      fullPath = local_path_planner->process_new_array(track);
      publish_track_points();
      delete track;
    }

    /**
     * Publisher point by point
     */
    void publish_track_points() const {
      auto message = custom_interfaces::msg::PointArray();
      for (size_t i = 0; i < fullPath.size(); i++) {
        auto point = custom_interfaces::msg::Point2d();
        point.x = fullPath[i]->getX();
        point.y = fullPath[i]->getY();
        message.points.push_back(point);
      }
      RCLCPP_INFO(this->get_logger(), "Published message size = %ld", message.points.size());
      global_pub_->publish(message);
    }

    rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr vl_sub_;
    rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr track_sub_;
    rclcpp::Publisher<custom_interfaces::msg::PointArray>::SharedPtr local_pub_;
    rclcpp::Publisher<custom_interfaces::msg::PointArray>::SharedPtr global_pub_;
    size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planning>());
  rclcpp::shutdown();
  return 0;
}