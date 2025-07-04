#pragma once

#include <memory>

#include "common_lib/car_parameters/car_parameters.hpp"

class LoadTransferModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  LoadTransferModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}
};
