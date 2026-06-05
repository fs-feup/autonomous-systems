#pragma once

#include "base_tire_model.hpp"
#include "common_lib/structures/force.hpp"
#include "common_lib/structures/velocities.hpp"
#include "common_lib/structures/wheels.hpp"

/*
-------------------------------------------------------------------------------------------
  Only struct to be declared here since it is suposed to be used only for this tire model
-------------------------------------------------------------------------------------------
*/
struct InternalValues {
  double epsilong;
  double epsilon = 1e-6;  // safety factor to avoid division by zero
  double dfz;
  double Fz0_prime;
  double Vc;
  double Vc_prime;
  double alpha_prime;
  double phi;
  double dpi = 0;  // defaulted to 0 (ignore pressure changes)
  double gamma_star;
  double Kya;
  double Kya_prime;
  double Vcx;
  double longitudinal_direction = 1.0;
  double LMUY_prime;
  double SVyg;
  double alpha_star;
  double zeta0 = 1;  // This is an on and off switch this is used to consider the effects of
                     // turnslip (1 means no turn slip effects, 0 means turn slip effects on)
  double zeta1;
  double zeta2;
  double zeta3;
  double zeta7;
  double zeta8;
  double SHyp;
  double SVyk;
  double Kyg0;
  double muy;
  double kxk;
};

//------------------------------------------------------------------------------------------------------------------------------------------------
// IMPORTANT: In case of wanting to account for pressure changes, there is currently no
// implementation, we always assume constant tire pressure
//------------------------------------------------------------------------------------------------------------------------------------------------
class PacejkaMF6_2 : public TireModel {
public:
  explicit PacejkaMF6_2(const common_lib::car_parameters::CarParameters& car_parameters)
      : TireModel(car_parameters){};

  Eigen::Vector4d tire_forces(const TireInput& tire_input) override;

private:
  InternalValues internal_vals;

  /**
   * @brief Calculates the shift for the slip ratio calculation
   *
   * @return double Shift
   */
  double calculate_SHx() const;

  /**
   * @brief Calculates the shift for the slip angle calculation
   *
   * @return double Shift
   */
  double calculate_SHy() const;

  /**
   * @brief Uses the pacejka magic formula to calculate either Fx or Fy depending on the input
   * values
   *
   * @param B Stiffness
   * @param C Shape
   * @param D Peak value
   * @param E Fall-off
   * @param slip slip angle or slip ratio
   * @param SV Vertical Shift
   * @return double Either Fx or Fy depeding on inputs
   */
  double calculate_pure_slip(double B, double C, double D, double E, double shifted_slip,
                             double SV) const;

  /**
   * @brief Intermediate function that calculates all the internal values needed for future
   * calculations.
   *
   * @param slip_angle Slip angle
   * @param slip_ratio Slip ratio
   * @param vertical_load Fz
   * @param contact_patch_longitudinal_velocity Longitudinal velocity in the wheel frame
   * @param contact_patch_lateral_velocity Lateral velocity in the wheel frame
   * @param yaw_rate Yaw rate
   * @param wheel_angular_speed Angular speed at the wheel
   * @param camber_angle Camber angle
   */
  void calculate_tire_state(double slip_angle, double slip_ratio, double vertical_load,
                            double contact_patch_longitudinal_velocity,
                            double contact_patch_lateral_velocity, double yaw_rate,
                            double wheel_angular_speed, double camber_angle);

  /**
   * @brief Calculates the D parameter for the Fy calculation using pacejka MF (Peak)
   *
   * @return double The value of Dy
   */
  double calculate_Dy(double vertical_load) const;

  double calculate_Dx(double vertical_load) const;

  double calculate_Cy(double vertical_load) const;
  /**

   * @brief Calculates the C component for the pure longitudinal slip (4.E11)
   *
   * @return double value of Cx
   */
  double calculate_Cx(double vertical_load) const;

  double calculate_Bx(double Dx, double Cx) const;

  double calculate_By(double Dy, double Cy) const;

  double calculate_Ey(double slip_angle) const;
  /**
   * @brief Calculates the E component for the longitudinal slip
   *
   * @param shifted_slip_ratio The already shited slip ratio (kappax)
   * @return double Value of Ex
   */
  double calculate_Ex(double shifted_slip_ratio) const;

  double calculate_SVy(double vertical_load) const;

  double calculate_SVx(double vertical_load) const;

  /**
   * @brief Calculates Fx using combined slip
   *
   * @param Fx0 Fx calculated using pure slip
   * @param slip_ratio Slip ratio (kappa)
   * @return double Fx using combined slip
   */
  double calculate_combined_longitudinal(double Fx0, double slip_ratio) const;

  /**
   * @brief Calculates Fy using combined slip
   *
   * @param Fy0  Fy calculated using pure slip
   * @param shifted_slip_ratio Slip ratio already shifted (kappax)
   * @param slip_ratio Slip ratio (kappa)
   * @param vertical_load Fz
   * @return double Fy using combined slip
   */
  double calculate_combined_lateral(double Fy0, double shifted_slip_ratio, double slip_ratio,
                                    double vertical_load) const;

  /**
   * @brief [!NOT USED] Calculates the aligning moment without using combined slip
   *
   * @param Fy0 Fy calculated using pure slip
   * @param normal_load Normal load on the tire (Fz)
   * @param SHy Horizontal shift for slip angle
   * @param SVy Vertical shift for slip angle
   * @param By B component for slip angle
   * @param Cy C component for slip angle
   * @return double Self aligning moment without using combined slip
   */
  double calculate_pure_moment(double Fy0, double normal_load, double SHy, double SVy, double By,
                               double Cy, double longitudinal_velocity) const;

  /**
   * @brief Calculates the aligning moment using combined slip.
   *
   * @param Fx Fx calculated using combined slip
   * @param Fy Fy calculated using combined slip
   * @param slip_ratio Slip ratio (kappa)
   * @param SHy Horizontal shift for slip angle
   * @param SVy Vertical shift for slip angle
   * @param By B component for slip angle
   * @param Cy C component for slip angle
   * @return double Self aligning moment using combined slip
   */
  double calculate_combined_moment(double Fx, double Fy, double slip_ratio, double SHy, double SVy,
                                   double By, double Cy, double longitudinal_velocity,
                                   double normal_load) const;

  /**
   * @brief Calculates the rolling resistance moment using the longitudinal slip and vertical load
   *
   * @param vertical_load Vertical load on the tire (Fz)
   * @param Fx Longitudinal force on the tire
   * @param vx Longitudinal velocity of the tire
   * @return double Rolling resistance moment
   */
  double calculate_rolling_resistance_moment(double vertical_load, double Fx, double vx) const;

  double calculate_Zeta1(double slip_ratio) const;

  double calculate_Zeta2(double slip_angle) const;

  double calculate_Zeta3() const;

  double calculate_SHyp(double camber_angle) const;

  double calculate_SVyk(double vertical_load, double slip_ratio) const;
};
