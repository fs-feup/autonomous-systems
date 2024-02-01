#include "custom_interfaces/msg/cone_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class Perception : public rclcpp::Node {
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _point_cloud_subscription;
  rclcpp::Publisher<custom_interfaces::msg::ConeArray>::SharedPtr _cones_publisher;

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

 public:
  Perception();
};