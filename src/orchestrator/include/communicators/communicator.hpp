#ifndef SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_COMMUNICATOR_HPP_
#define SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_COMMUNICATOR_HPP_

#include "custom_interfaces/msg/vcu_command.hpp"
#include "custom_interfaces/msg/vcu.hpp"

class Communicator {
 public:
  virtual void send_to_car(const custom_interfaces::msg::VcuCommand msg) = 0;
};

#endif  // SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_COMMUNICATOR_HPP_