#include "../../../include/vehicle_model/tire_model/tire_model.hpp"

#include <cmath>

TireModel::TireModel() {
  tireParameters_ = {0.0, 0.0};
  pacejka = {1.0, 1.0, 1.0, 1.0};
}

TireModel::TireModel(float effective_radius, float distance_to_cg, PacejkaParameters pacejka_) {
  TireParameters tireParameters = {effective_radius, distance_to_cg};
  tireParameters_ = tireParameters;
  pacejka = pacejka_;
}

float TireModel::calculateSlipAngle(float v_x, float v_y, float yawR) const {
  if (v_x == 0.0) v_x = 0.1;
  float numerator = v_y + (tireParameters_.distanceToCG * yawR);
  return atan2(numerator, v_x);  // Aparentemente atan2 é melhor do que atan normal
}

float TireModel::calculateSlipRatio(float angularSpeed, float v_x) const {
  if (v_x == 0.0) v_x = 0.1;
  float numerator = (tireParameters_.effectiveTireR * angularSpeed) - v_x;
  return numerator / v_x;
}

TireOutput TireModel::calculateForces(const TireInput& input) const {
  float slipA = calculateSlipAngle(input.v.vx, input.v.vy, input.yawR);
  float slipR = calculateSlipRatio(input.angSpeed, input.v.vx);
  TireOutput out{};
  out.Fx = calculatePacejka(slipR);
  out.Fy = calculatePacejka(slipA);
  return out;
}

float TireModel::calculatePacejka(float slip) const {
  float ba = pacejka.B * slip;
  float inner = pacejka.E * (ba - atan(ba));    // E(Ba - arctan(ba))
  float middle = pacejka.C * atan(ba - inner);  // C arctan(Ba- E(Ba - arctan(ba)))
  return pacejka.D * sin(middle);               // D sin(C arctan(Ba- E(Ba - arctan(ba))))
}

float TireModel::get_tire_effective_radius() const { return tireParameters_.effectiveTireR; };