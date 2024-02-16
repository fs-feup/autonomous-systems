#include "adapter/adapter.hpp"
#include "perception/perception_node.hpp"

Adapter::Adapter(std::string mode, Perception *perception) {
  this->node = perception;
  this->mode = mode;

  RCLCPP_INFO(this->node->get_logger(), "mode: %s", mode.c_str());

  if (mode == "fsds") {
    this->fsds_init();
  }
  else if (mode == "livox"){
    this->livox_init();
  }
}

void Adapter::fsds_init() {
  this->fsds_state_subscription_ = this->node->create_subscription<fs_msgs::msg::GoSignal>(
      "/signal/go", 10,
      std::bind(&Adapter::fsds_mission_state_callback, this, std::placeholders::_1));
  this->_point_cloud_subscription = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/Lidar1", 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {node->pointCloudCallback(msg);});

  this->fsds_ebs_publisher_ =
      this->node->create_publisher<fs_msgs::msg::FinishedSignal>("/signal/finished", 10);
}

void Adapter::livox_init(){
    this->_point_cloud_subscription = this->node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {node->pointCloudCallback(msg);});
}

void Adapter::fsds_mission_state_callback(const fs_msgs::msg::GoSignal msg) {
  // Mission is unnecessary
  // TODO: just info that the message is being received
  return;
}

void Adapter::fsds_finish(){
  auto message = fs_msgs::msg::FinishedSignal();
  message.placeholder = true; // unnecessary

  this->fsds_ebs_publisher_->publish(message);
}