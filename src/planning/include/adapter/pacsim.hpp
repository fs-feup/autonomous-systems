#ifndef SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_PACSIM_HPP_
#define SRC_PLANNING_PLANNING_INCLUDE_ADAPTER_PACSIM_HPP_

#include <empty__struct.hpp>

#include "adapter/adapter.hpp"
#include "std_srvs/srv/trigger.hpp"

class PacSimAdapter : public Adapter {
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr pacsim_ebs_server;
  void handle_service(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                      std::shared_ptr<std_srvs::srv::Empty::Response> response);

 public:
  explicit PacSimAdapter(Planning* planning);

  void init() override;
  void mission_state_callback();  // ?
  void set_mission_state(int mission, int state) override;
  void finish() override;
};

#endif