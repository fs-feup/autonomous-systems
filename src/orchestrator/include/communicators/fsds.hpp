#ifndef SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_FSDS_HPP_
#define SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_FSDS_HPP_

#include "communicators/communicator.hpp"
#include "custom_interfaces/msg/vehicle_command.hpp"
#include "custom_interfaces/msg/vehicle_info.hpp"
#include "orchestrator/orchestrator.hpp"

class FsdsCommunicator : public Communicator {
 public:
  FsdsCommunicator(Orchestrator* orchestrator);
  void send_to_car(const custom_interfaces::msg::VehicleCommand msg) override;
  void send_from_car();

 private:
  Orchestrator* orchestrator_;
};

#endif  // SRC_ORCHESTRATOR_INCLUDE_COMMUNICATORS_FSDS_HPP_