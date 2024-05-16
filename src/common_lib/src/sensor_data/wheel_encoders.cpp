#include "common_lib/sensor_data/wheel_encoders.hpp"

namespace common_lib::sensor_data {

WheelEncoderData::WheelEncoderData(double rl_rpm, double rr_rpm, double steering_angle)
    : rl_rpm(rl_rpm), rr_rpm(rr_rpm), steering_angle(steering_angle) {}

WheelEncoderData::WheelEncoderData(double rl_rpm, double rr_rpm, double fl_rpm, double fr_rpm,
                                   double steering_angle)
    : rl_rpm(rl_rpm),
      rr_rpm(rr_rpm),
      fl_rpm(fl_rpm),
      fr_rpm(fr_rpm),
      steering_angle(steering_angle) {}

}  // namespace common_lib::sensor_data