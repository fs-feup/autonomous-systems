// #include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "../include/planning/finalpathplanner.hpp"
#include "../include/planning/slampathplanner.hpp"

#include "custom_interfaces/msg/point2d.hpp"
#include "custom_interfaces/msg/point_array.hpp"
#include "custom_interfaces/msg/pose.hpp"
#include "custom_interfaces/msg/cone_array.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class Planning : public rclcpp::Node
{
  vector<pair<float, float>> fullPath; /**<path data container */
  SlamPathPlanner* slampathplanner  = new SlamPathPlanner();

  public:
    Planning()
    : Node("planning"), count_(0){
      vl_sub_ = this->create_subscription<custom_interfaces::msg::Pose>(
      "vehicle_localization", 10, std::bind(&Planning::vehicle_localisation_callback, this, _1));
    
      track_sub_ = this->create_subscription<custom_interfaces::msg::ConeArray>(
        "track_map", 10, std::bind(&Planning::track_map_callback, this, _1));
      
      slam_pub_ = this->create_publisher<custom_interfaces::msg::PointArray>("planning_slam", 10);
      endurance_pub_ = this->create_publisher<custom_interfaces::msg::PointArray>("planning_endurance", 10);
      
      std::string filePrefix = rcpputils::fs::current_path().string();
      std::string filePackage =  filePrefix + "/planning/planning/files/hairpins.txt";
      std::cout << filePackage;

      Track* track = new Track();
      track->fillTrack(filePackage);

      // FinalPathPlanner* finalpathplanner = new FinalPathPlanner(track);
      // finalpathplanner->middlePath();
      // finalpathplanner->writeFinalPath(filePrefix);
      // fullPath = finalpathplanner->getPath();
          
      publish_track_points(); 
    }

  private:
   
    void vehicle_localisation_callback(const custom_interfaces::msg::Pose msg) const {
      RCLCPP_INFO(this->get_logger(), "Received x = '%f' | y = '%f'", msg.position.x, msg.position.y);
    }

    void track_map_callback(const custom_interfaces::msg::ConeArray msg) const {
      RCLCPP_INFO(this->get_logger(), "Received Cone Array with size = '%ld'", msg.cone_array.size());
      Track track = new Track();
      for (int i = 0; i < msg.cone_array.size(); i++){
        auto cone = msg.cone_array[i];
        track.addCone(cone.position.x, cone.position.y, cone.color);
      }
      slampathplanner->processNewArray(track);  
    }

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
        message.points.push_back(point);
      }
      RCLCPP_INFO(this->get_logger(), "Publishing message with size %ld", message.points.size());
      endurance_pub_->publish(message);
    }

    rclcpp::Subscription<custom_interfaces::msg::Pose>::SharedPtr vl_sub_;
    rclcpp::Subscription<custom_interfaces::msg::ConeArray>::SharedPtr track_sub_;
    rclcpp::Publisher<custom_interfaces::msg::PointArray>::SharedPtr slam_pub_;
    rclcpp::Publisher<custom_interfaces::msg::PointArray>::SharedPtr endurance_pub_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Planning>());
  rclcpp::shutdown();
  return 0;
}