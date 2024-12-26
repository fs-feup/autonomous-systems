#include "adapters/pacsim_adapter.hpp"

PacsimAdapter::PacsimAdapter(const VEParameters& parameters) : Adapter(parameters) {
  cog_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/pacsim/imu/cog_imu", 1,
      std::bind(&PacsimAdapter::ImuCallback, this, std::placeholders::_1));
}

void PacsimAdapter::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  common_lib::sensor_data::ImuData imu_data;
  imu_data.acceleration_x = msg->linear_acceleration.x;
  imu_data.acceleration_y = msg->linear_acceleration.y;
  imu_data.rotational_velocity = msg->angular_velocity.z;
  this->_velocity_estimator_->IMUCallback(imu_data);
}