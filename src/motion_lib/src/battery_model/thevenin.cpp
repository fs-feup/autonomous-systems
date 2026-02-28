#include "motion_lib/battery_model/thevenin.hpp"

Thevenin::Thevenin(const common_lib::car_parameters::CarParameters& car_parameters)
    : BatteryModel(car_parameters), soc_(0.88f), v_rc_(0.0f), current_(0.0f) {}

std::tuple<float, float> Thevenin::calculate_current_for_power(float electrical_power_req) const {
  float ocv_cell = get_cell_ocv(soc_);
  float r0_cell = get_cell_r0(soc_);

  float ocv_eff_pack = (ocv_cell - v_rc_) * this->car_parameters_->battery_parameters->cells_series;
  float r0_pack =
      r0_cell * (static_cast<float>(this->car_parameters_->battery_parameters->cells_series) /
                 this->car_parameters_->battery_parameters->cells_parallel);

  float discriminant = std::pow(ocv_eff_pack, 2) - 4.0f * r0_pack * electrical_power_req;

  float calculated_current;
  if (discriminant < 0.0f) {
    calculated_current = ocv_eff_pack / (2.0f * r0_pack);

    // LOG 1: Debugging why the power request failed
    RCLCPP_WARN(rclcpp::get_logger("Thevenin"),
                "LIMIT HIT: Req %0.2f W | Max Avail %0.2f W | SoC: %0.2f | OCV_eff_pack: %0.2f V | "
                "R0_pack: %0.4f Ohm",
                electrical_power_req, (std::pow(ocv_eff_pack, 2) / (4.0f * r0_pack)), soc_,
                ocv_eff_pack, r0_pack);
  } else {
    calculated_current = (ocv_eff_pack - std::sqrt(discriminant)) / (2.0f * r0_pack);
  }

  calculated_current = std::min(calculated_current,
                                this->car_parameters_->battery_parameters->max_discharge_current);

  float terminal_voltage = ocv_eff_pack - (calculated_current * r0_pack);

  if (terminal_voltage < this->car_parameters_->battery_parameters->min_voltage) {
    calculated_current =
        (ocv_eff_pack - this->car_parameters_->battery_parameters->min_voltage) / r0_pack;
    terminal_voltage = this->car_parameters_->battery_parameters->min_voltage;
  }

  return std::make_tuple(calculated_current, terminal_voltage * calculated_current);
}

float Thevenin::get_current() const { return current_; }

float Thevenin::get_voltage() const {
  float ocv_cell = get_cell_ocv(soc_);
  float r0_cell = get_cell_r0(soc_);

  // 1. Scale cell parameters to full pack level
  float pack_ocv = ocv_cell * this->car_parameters_->battery_parameters->cells_series;
  float pack_v_rc = v_rc_ * this->car_parameters_->battery_parameters->cells_series;

  // Resistance Scaling: (R_cell * N_series) / N_parallel
  float pack_r0 =
      r0_cell * (static_cast<float>(this->car_parameters_->battery_parameters->cells_series) /
                 this->car_parameters_->battery_parameters->cells_parallel);

  // 2. Terminal Voltage = Pack_OCV - (Pack_Current * Pack_R0) - Pack_V_RC
  // current_ is the total pack current (e.g., 175A)
  return pack_ocv - (current_ * pack_r0) - pack_v_rc;
}

float Thevenin::get_voltage(float current_draw) const {
  // Get individual cell parameters based on current SoC
  float ocv_cell = get_cell_ocv(soc_);
  float r0_cell = get_cell_r0(soc_);

  // 1. Scale cell parameters to full pack level
  // OCV and V_rc are additive in series
  float pack_ocv = ocv_cell * this->car_parameters_->battery_parameters->cells_series;
  float pack_v_rc = v_rc_ * this->car_parameters_->battery_parameters->cells_series;

  // 2. Scale Resistance: (R_cell * N_series) / N_parallel
  float pack_r0 =
      r0_cell * (static_cast<float>(this->car_parameters_->battery_parameters->cells_series) /
                 this->car_parameters_->battery_parameters->cells_parallel);

  // 3. Terminal Voltage = Pack_OCV - (current_draw * Pack_R0) - Pack_V_RC
  // current_draw is the total pack current provided as an argument
  return pack_ocv - (current_draw * pack_r0) - pack_v_rc;
}

float Thevenin::get_open_circuit_voltage() const {
  return get_cell_ocv(soc_) * this->car_parameters_->battery_parameters->cells_series;
}

void Thevenin::update_state(float current_draw, float dt) {
  // Update SoC (Coulomb Counting)
  soc_ -= (current_draw * dt) / (3600.0f * this->car_parameters_->battery_parameters->capacity_ah);
  soc_ = std::clamp(soc_, 0.0f, 1.0f);

  // Update Polarization Voltage (V_rc)
  float r1 = get_cell_r1(soc_);
  float c1 = get_cell_c1(soc_);
  float r0 = get_cell_r0(soc_);    // Get R0 for logging
  float ocv = get_cell_ocv(soc_);  // Get OCV for logging

  float tau = std::max(r1 * c1, 1e-6f);
  float i_cell =
      current_draw / static_cast<float>(this->car_parameters_->battery_parameters->cells_parallel);

  float ak = std::exp(-dt / tau);
  float bk = r1 * (1.0f - ak);

  this->v_rc_ = ak * this->v_rc_ + bk * i_cell;
  this->current_ = current_draw;
}

float Thevenin::get_soc() const { return soc_; }

// Polynomial Solver
// Horner's Method: (((a5*x + a4)*x + a3)*x + a2)*x + a1)*x + a0
float Thevenin::solve_polinomial5(const std::vector<float>& c, float x) const {
  return ((((c[0] * x + c[1]) * x + c[2]) * x + c[3]) * x + c[4]) * x + c[5];
}

// Cell Parameter Getters
float Thevenin::get_cell_ocv(float soc) const {
  return solve_polinomial5({this->car_parameters_->battery_parameters->OCV_a5,
                            this->car_parameters_->battery_parameters->OCV_a4,
                            this->car_parameters_->battery_parameters->OCV_a3,
                            this->car_parameters_->battery_parameters->OCV_a2,
                            this->car_parameters_->battery_parameters->OCV_a1,
                            this->car_parameters_->battery_parameters->OCV_a0},
                           soc);
}

float Thevenin::get_cell_r0(float soc) const {
  return solve_polinomial5({this->car_parameters_->battery_parameters->R0_a5,
                            this->car_parameters_->battery_parameters->R0_a4,
                            this->car_parameters_->battery_parameters->R0_a3,
                            this->car_parameters_->battery_parameters->R0_a2,
                            this->car_parameters_->battery_parameters->R0_a1,
                            this->car_parameters_->battery_parameters->R0_a0},
                           soc);
}

float Thevenin::get_cell_r1(float soc) const {
  return solve_polinomial5({this->car_parameters_->battery_parameters->R1_a5,
                            this->car_parameters_->battery_parameters->R1_a4,
                            this->car_parameters_->battery_parameters->R1_a3,
                            this->car_parameters_->battery_parameters->R1_a2,
                            this->car_parameters_->battery_parameters->R1_a1,
                            this->car_parameters_->battery_parameters->R1_a0},
                           soc);
}

float Thevenin::get_cell_c1(float soc) const {
  return solve_polinomial5({this->car_parameters_->battery_parameters->C1_a5,
                            this->car_parameters_->battery_parameters->C1_a4,
                            this->car_parameters_->battery_parameters->C1_a3,
                            this->car_parameters_->battery_parameters->C1_a2,
                            this->car_parameters_->battery_parameters->C1_a1,
                            this->car_parameters_->battery_parameters->C1_a0},
                           soc);
}

// Reset battery state
void Thevenin::reset() {
  soc_ = 1.0f;
  current_ = 0.0f;
  v_rc_ = 0.0f;
}