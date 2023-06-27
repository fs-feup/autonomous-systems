#ifndef SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_EUFS_HPP_
#define SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_EUFS_HPP_

#include "communicators/communicator.hpp"
#include "custom_interfaces/msg/vehicle_info.hpp"

class EufsCommunicator : public Communicator {
 public:
  EufsCommunicator(Orchestrator* orchestrator);
  void send_to_car();
  custom_interfaces::msg::VehicleInfo read_data();
};

#endif  // SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_EUFS_HPP_