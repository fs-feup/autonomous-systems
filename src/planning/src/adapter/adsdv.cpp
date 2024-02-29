#include "adapter/adsdv.hpp"
#include "planning/planning.hpp"

AdsdvAdapter::AdsdvAdapter(Planning* planning) : Adapter(planning) {
  this->init();
}

void AdsdvAdapter::init() {
  this->ads_dv_state_subscription_ = this->node->create_subscription<custom_interfaces::msg::Vcu>(
      "/vcu", 10, std::bind(&AdsdvAdapter::mission_state_callback, this, std::placeholders::_1));
}

void AdsdvAdapter::mission_state_callback(custom_interfaces::msg::Vcu msg) {
  auto mission = msg.ami_state;
  this->node->set_mission(adsdvToSystem.at(mission)); // map adsdv mission to system mission
}

void AdsdvAdapter::set_mission_state(int mission, int state) {
    std::cout << "Set mission undefined for Ads Dv\n";
}

void AdsdvAdapter::finish() {
  std::cout << "Finish undefined for Ads Dv\n";
}