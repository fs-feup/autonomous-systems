#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADSDV_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_ADSDV_HPP_

#include "adapter/adapter.hpp"

class Planning;

class AdsdvAdapter : public Adapter {
    rclcpp::Subscription<custom_interfaces::msg::Vcu>::SharedPtr ads_dv_state_subscription_;

 public:
    explicit AdsdvAdapter(Planning* planning);

    void init() override;
    void mission_state_callback(custom_interfaces::msg::Vcu msg);
    void set_mission_state(int mission, int state) override;
    void finish() override;
};

#endif