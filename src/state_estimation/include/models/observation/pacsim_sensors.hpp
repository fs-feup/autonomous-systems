#pragma once

#include "models/observation/observation_model.hpp"
#include "motion_lib/steering_model/map.hpp"
#include "motion_lib/tire_model/map.hpp"

class ObservationModelPacsim : public ObservationModel {
  enum SensorId : std::size_t {
    IMU = 0,   // rows 0,1,2 (ax, ay, yaw_rate)
    WSS_FL,    // rows 3 (state-based), 7 (kinematic)
    WSS_FR,    // rows 4, 8
    WSS_RL,    // rows 5, 9
    WSS_RR,    // rows 6, 10
    STEERING,  // row 11
    NUM_SENSORS
  };

  common_lib::sensor_data::ImuData last_imu_data_ = common_lib::sensor_data::ImuData();
  common_lib::sensor_data::WheelEncoderData last_wss_data_ =
      common_lib::sensor_data::WheelEncoderData();
  double last_steering_angle_ = 0.0;
  std::shared_ptr<SteeringModel> steering_model_;
  std::shared_ptr<TireModel> tire_model_;

  static std::vector<SensorSpec> build_specs(const std::shared_ptr<SEParameters>& parameters);
  static std::vector<std::size_t> build_row_to_sensor();

public:
  ObservationModelPacsim(const std::shared_ptr<SEParameters>& parameters)
      : ObservationModel(parameters, build_specs(parameters), build_row_to_sensor()) {
    this->steering_model_ = steering_models_map.at(this->parameters_->steering_model_name_)(
        this->parameters_->car_parameters_);
    this->tire_model_ =
        tire_models_map.at(this->parameters_->tire_model_name_)(this->parameters_->car_parameters_);
  }

  /**
   * @brief Transforms the state into the Pacsim's sensor observation space, in the order
   * [ax, ay, yaw_rate, fl/fr/rl/rr state-based, fl/fr/rl/rr kinematic, steering_angle].
   */
  void expected_observations(const State& state,
                             Eigen::Ref<Eigen::VectorXd> expected_observations) override;

  void update_imu_data(const common_lib::sensor_data::ImuData& imu_data) override;
  void update_wss_data(const common_lib::sensor_data::WheelEncoderData& wss_data) override;
  void update_motor_rpm(double /*motor_rpm*/, const rclcpp::Time& /*stamp*/) override {}
  void update_steering_angle(double steering_angle, const rclcpp::Time& stamp) override;

  Eigen::VectorXd get_last_observations() const override;
  Eigen::MatrixXd get_last_observations_noise() const override;
};
