#ifndef SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_ADSDV_HPP_
#define SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_ADSDV_HPP_

#include "communicators/communicator.hpp"
#include "custom_interfaces/msg/vehicle_info.hpp"

class AdsDvCommunicator : public Communicator {
 public:
  AdsDvCommunicator();
  void send_to_car();
  custom_interfaces::msg::VehicleInfo read_from_car();
};

#endif  // SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_ADSDV_HPP_