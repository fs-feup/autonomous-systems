#include "motion_lib/battery_model/thevenin.hpp"

Thevenin::Thevenin(const common_lib::car_parameters::CarParameters& car_parameters)
    : BatteryModel(car_parameters),
      soc_(car_parameters.battery_parameters->initial_soc),
      v_rc_(0.0f),
      current_(0.0f) {}

float Thevenin::calculate_allowed_current(float requested_current) const {
  float ocv_cell = get_cell_ocv(soc_);
  float r0_cell = get_cell_r0(soc_);

  // Calculate pack-level parameters
  float ocv_eff_pack = (ocv_cell - v_rc_) * this->car_parameters_->battery_parameters->cells_series;
  float r0_pack =
      r0_cell * (static_cast<float>(this->car_parameters_->battery_parameters->cells_series) /
                 this->car_parameters_->battery_parameters->cells_parallel);

  // FSG rules 80kW limit
  float disc_p = std::pow(ocv_eff_pack, 2) - 4.0f * r0_pack * 80000.0f;
  float i_limit_80kw = (disc_p < 0.0f) ? (ocv_eff_pack / (2.0f * r0_pack))
                                       : ((ocv_eff_pack - std::sqrt(disc_p)) / (2.0f * r0_pack));

  // Minimum voltage limit
  float i_limit_v_min =
      (ocv_eff_pack - this->car_parameters_->battery_parameters->min_voltage) / r0_pack;

  // Maximum discharge current
  float i_limit_hardware = this->car_parameters_->battery_parameters->max_discharge_current;

  // Calculate the maximum allowed current based on all limits
  float max_allowed = std::min({i_limit_80kw, i_limit_v_min, i_limit_hardware});

  // Return the minimum of the requested current and the maximum allowed current
  return std::min(requested_current, max_allowed);
}

float Thevenin::get_current() const { return current_; }

float Thevenin::get_voltage() const {
  float ocv_cell = get_cell_ocv(soc_);
  float r0_cell = get_cell_r0(soc_);

  // Calculate pack-level parameters
  float pack_ocv = ocv_cell * this->car_parameters_->battery_parameters->cells_series;
  float pack_v_rc = v_rc_ * this->car_parameters_->battery_parameters->cells_series;
  float pack_r0 =
      r0_cell * (static_cast<float>(this->car_parameters_->battery_parameters->cells_series) /
                 this->car_parameters_->battery_parameters->cells_parallel);

  // Terminal Voltage = Pack_OCV - (Pack_Current * Pack_R0) - Pack_V_RC
  return pack_ocv - (current_ * pack_r0) - pack_v_rc;
}

float Thevenin::get_voltage(float current_draw) const {
  float ocv_cell = get_cell_ocv(soc_);
  float r0_cell = get_cell_r0(soc_);

  // Calculate pack-level parameters
  float pack_ocv = ocv_cell * this->car_parameters_->battery_parameters->cells_series;
  float pack_v_rc = v_rc_ * this->car_parameters_->battery_parameters->cells_series;
  float pack_r0 =
      r0_cell * (static_cast<float>(this->car_parameters_->battery_parameters->cells_series) /
                 this->car_parameters_->battery_parameters->cells_parallel);

  // Terminal Voltage = Pack_OCV - (Current_Request * Pack_R0) - Pack_V_RC
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
  soc_ = car_parameters_->battery_parameters->initial_soc;
  current_ = 0.0f;
  v_rc_ = 0.0f;
}