#include "models/process/component_based_vehicle_model.hpp"

ComponentBasedVehicleModel::ComponentBasedVehicleModel(
    const std::shared_ptr<SEParameters>& parameters)
    : ProcessModel(parameters) {
  // Initialize the models based on the parameters
}

void ComponentBasedVehicleModel::predict(Eigen::Ref<State> state,
                                         common_lib::structures::ControlCommand control_command,
                                         double dt){
    // TODO: implement vm
};