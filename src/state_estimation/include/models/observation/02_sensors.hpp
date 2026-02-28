#pragma once

#include "models/observation/observation_model.hpp"

class ObservationModel02 : public ObservationModel {
public:
  void transform(State& state, common_lib::sensor_data::ImuData imu_data,
                 common_lib::sensor_data::WheelEncoderData wheel_encoder_data,
                 double steering_angle, double motor_rpm) override;
};