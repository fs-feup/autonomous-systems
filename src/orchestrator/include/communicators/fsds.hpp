#ifndef SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_FSDS_HPP_
#define SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_FSDS_HPP_

#include "communicators/communicator.hpp"
#include "custom_interfaces/msg/vehicle_info.hpp"

class FsdsCommunicator : public Communicator {
 public:
  FsdsCommunicator();
  void send_to_car();
  custom_interfaces::msg::VehicleInfo read_data();
};

#endif  // SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_FSDS_HPP_