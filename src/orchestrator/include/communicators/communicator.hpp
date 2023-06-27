#ifndef SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_COMMUNICATOR_HPP_
#define SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_COMMUNICATOR_HPP_

#include "custom_interfaces/msg/vehicle_info.hpp"

class Communicator {
 public:
  Communicator();
  virtual void send_to_car() = 0;
  virtual custom_interfaces::msg::VehicleInfo read_from_car() = 0;
};

#endif  // SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_COMMUNICATOR_HPP_