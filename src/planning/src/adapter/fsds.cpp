#include "adapter/fsds.hpp"

#include "planning/planning.hpp"

FsdsAdapter::FsdsAdapter(Planning* planning) : Adapter(planning) { this->init(); }

void FsdsAdapter::init() {
  this->fsds_state_subscription_ = this->node->create_subscription<fs_msgs::msg::GoSignal>(
      "/signal/go", 10,
      std::bind(&FsdsAdapter::mission_state_callback, this, std::placeholders::_1));
  this->fsds_ebs_publisher_ =
      this->node->create_publisher<fs_msgs::msg::FinishedSignal>("/signal/finished", 10);
}

void FsdsAdapter::mission_state_callback(const fs_msgs::msg::GoSignal msg) {
  auto mission = msg.mission;
  // map fsds mission to system mission
  this->node->set_mission(fsdsToSystem.at(mission));
}

void FsdsAdapter::set_mission_state(int mission, int state) {
  std::cout << "Set mission undefined for Eufs\n";
}

void FsdsAdapter::finish() {
  auto message = fs_msgs::msg::FinishedSignal();
  message.placeholder = true;  // unnecessary

  this->fsds_ebs_publisher_->publish(message);
}
