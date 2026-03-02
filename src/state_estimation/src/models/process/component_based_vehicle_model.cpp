#include "models/process/component_based_vehicle_model.hpp"

ComponentBasedVehicleModel::ComponentBasedVehicleModel(
    const std::shared_ptr<SEParameters>& parameters)
    : ProcessModel(parameters) {
  // Initialize the models based on the parameters
  this->aero_model_ = aero_models_map.at(parameters->aero_model_name_)(parameters->car_parameters_);
  this->load_transfer_model_ = load_transfer_models_map.at(parameters->load_transfer_model_name_)(
      parameters->car_parameters_);
  this->steering_model_ =
      steering_models_map.at(parameters->steering_model_name_)(parameters->car_parameters_);
}

void ComponentBasedVehicleModel::predict(Eigen::Ref<State> state,
                                         common_lib::structures::ControlCommand control_command,
                                         double dt){
    // TODO: implement vm
};