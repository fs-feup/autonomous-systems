#pragma once

#include "models/observation/observation_model.hpp"

class ObservationModelPacsim : public ObservationModel {
public:
  void transform(State& state, common_lib::sensor_data::ImuData imu_data,
                 common_lib::sensor_data::WheelEncoderData wheel_encoder_data,
                 double steering_angle, [[maybe_unused]] double motor_rpm = 0) override;
};