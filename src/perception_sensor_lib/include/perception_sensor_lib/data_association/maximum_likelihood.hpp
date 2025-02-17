#pragma once

#include "perception_sensor_lib/data_association/base_data_association.hpp"

class MaximumLikelihood : public DataAssociationModel {
public:
  MaximumLikelihood() = default;
  ~MaximumLikelihood() = default;
};