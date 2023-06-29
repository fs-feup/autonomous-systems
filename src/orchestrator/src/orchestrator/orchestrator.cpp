#include "orchestrator/orchestrator.hpp"

#include "communicators/ads-dv.hpp"
#include "communicators/eufs.hpp"
#include "communicators/fsds.hpp"
#include "custom_interfaces/msg/gps.hpp"
#include "custom_interfaces/msg/imu.hpp"
#include "custom_interfaces/msg/odom.hpp"
#include "custom_interfaces/msg/state.hpp"
#include "custom_interfaces/msg/vcu.hpp"
#include "custom_interfaces/msg/vcu_command.hpp"
#include "rclcpp/rclcpp.hpp"

Orchestrator::Orchestrator(const std::string& mode) : Node("orchestrator") {
  if (mode == "ads-dv") {
    this->communicator_ = new AdsDvCommunicator(this);
  } else if (mode == "eufs") {
    this->communicator_ = new EufsCommunicator(this);
  } else if (mode == "fsds") {
    this->communicator_ = new FsdsCommunicator(this);
  } else {
    printf("Invalid mode!\r\n");
  }

  this->vcu_publisher_ = this->create_publisher<custom_interfaces::msg::Vcu>("vcu", 10);
  this->imu_publisher_ = this->create_publisher<custom_interfaces::msg::Imu>("imu", 10);
  this->gps_publisher_ = this->create_publisher<custom_interfaces::msg::Gps>("gps", 10);
  this->state_publisher_ = this->create_publisher<custom_interfaces::msg::State>("gps", 10);
  this->odom_publisher_ = this->create_publisher<custom_interfaces::msg::Odom>("gps", 10);
  this->command_subscriber_ = this->create_subscription<custom_interfaces::msg::VcuCommand>(
      "vcu_command", 10, std::bind(&Orchestrator::send_to_car, this, std::placeholders::_1));
}

void Orchestrator::send_to_car(custom_interfaces::msg::VcuCommand msg) {
  this->communicator_->send_to_car(msg);
}

void Orchestrator::publish_vcu(custom_interfaces::msg::Vcu msg) {
  this->vcu_publisher_->publish(msg);
}

void Orchestrator::publish_state(custom_interfaces::msg::State msg) {
  this->state_publisher_->publish(msg);
}

void Orchestrator::publish_odom(custom_interfaces::msg::Odom msg) {
  this->odom_publisher_->publish(msg);
}

void Orchestrator::publish_imu(custom_interfaces::msg::Imu msg) {
  this->imu_publisher_->publish(msg);
}

void Orchestrator::publish_gps(custom_interfaces::msg::Gps msg) {
  this->gps_publisher_->publish(msg);
}