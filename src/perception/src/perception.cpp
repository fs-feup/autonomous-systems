#include "perception/perception.hpp"
#include <cstdio>


Perception::Perception() : Node("perception"){

  this->_point_cloud_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar/point_cloud",
    10,
    std::bind(&Perception::pointCloudCallback, this, std::placeholders::_1)
  );

  this->_cones_publisher = this->create_publisher<custom_interfaces::msg::ConeArray>(
    "cones",
    10
  );

}

void Perception::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  // TODO
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Perception>();

  RCLCPP_INFO(node->get_logger(), "Perception is alive!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
