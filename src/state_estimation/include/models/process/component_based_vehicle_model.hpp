#pragma once

#include "models/process/process_model.hpp"
#include "utils/parameters.hpp"

class ComponentBasedVehicleModel : public ProcessModel {
private:
public:
  ComponentBasedVehicleModel(const std::shared_ptr<SEParameters>& parameters);

  void predict(Eigen::Ref<State> state, common_lib::structures::ControlCommand control_command,
               double dt) override;
};