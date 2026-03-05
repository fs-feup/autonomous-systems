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

  // Esta definição é precisa para que ele compile, tens de mudar isto para o que for preciso como
  // input e mudar o base model, não sei muito bem como conciliar este com o pacejka_combined_slip,
  // mas o ideal é que ambos usem a mesma interface
  Eigen::Vector3d tire_forces(const TireInput& tire_input) override;

private:
  InternalValues internal_vals;

  /**
   * @brief Calculates the shift for the slip ratio calculation
   *
   * @return float Shift
   */
  float calculateSHx() const;

  /**
   * @brief Calculates the shift for the slip angle calculation
   *
   * @return float Shift
   */
  float calculateSHy() const;

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
   * @return float Either Fx or Fy depeding on inputs
   */
  float calculatePureSlip(float B, float C, float D, float E, float shifted_slip, float SV) const;

  /**
   * @brief Intermediate function that calculates all the internal values needed for future
   * calculations.
   *
   * @param slip_angle Slip angle
   * @param slip_ratio Slip ratio
   * @param vertical_load Fz
   * @param vx Velocity in x direction
   * @param vy Velocity in y direction
   * @param yaw_rate Yaw rate
   * @param wheel_angular_speed Angular speed at the wheel
   * @param steering_angle Steering angle
   * @param distance_to_CG Distance from the wheel to the center of gravity
   * @param camber_angle Camber angle
   * @return true Successful calculation of internal values
   */
  bool calculateTireState(float slip_angle, float slip_ratio, double vertical_load, double vx,
                          double vy, double yaw_rate, double wheel_angular_speed,
                          double steering_angle, double distance_to_CG, double camber_angle);

  /**
   * @brief Calculates the D parameter for the Fy calculation using pacejka MF (Peak)
   *
   * @return float The value of Dy
   */
  float calculateDy(double vertical_load) const;

  float calculateDx(double vertical_load) const;

  float calculateCy(double vertical_load) const;
  /**

   * @brief Calculates the C component for the pure longitudinal slip (4.E11)
   *
   * @return float value of Cx
   */
  float calculateCx(double vertical_load) const;

  float calculateBx(float Dx, float Cx) const;

  float calculateBy(float Dy, float Cy) const;

  float calculateEy(double slip_angle) const;
  /**
   * @brief Calculates the E component for the longitudinal slip
   *
   * @param shifted_slip_ratio The already shited slip ratio (kappax)
   * @return float Value of Ex
   */
  float calculateEx(double shifted_slip_ratio) const;

  float calculateSVy(double vertical_load) const;

  float calculateSVx(double vertical_load) const;

  /**
   * @brief Calculates Fx using combined slip
   *
   * @param Fx0 Fx calculated using pure slip
   * @param slip_ratio Slip ratio (kappa)
   * @return float Fx using combined slip
   */
  float calculateCombinedLongitudinal(double Fx0, double slip_ratio) const;

  /**
   * @brief Calculates Fy using combined slip
   *
   * @param Fy0  Fy calculated using pure slip
   * @param shifted_slip_ratio Slip ratio already shifted (kappax)
   * @param slip_ratio Slip ratio (kappa)
   * @param vertical_load Fz
   * @return float Fy using combined slip
   */
  float calculateCombinedLateral(double Fy0, double shifted_slip_ratio, double slip_ratio,
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
   * @return float Self aligning moment without using combined slip
   */
  float calculatePureMoment(double Fy0, double normal_load, double SHy, double SVy, double By,
                            double Cy, double vx) const;

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
   * @return float Self aligning moment using combined slip
   */
  float calculateCombinedMoment(double Fx, double Fy, double slip_ratio, double SHy, double SVy,
                                double By, double Cy, double vx, double normal_load) const;

private:
  float calculateZeta1(double slip_ratio) const;

  float calculateZeta2(double slip_angle) const;

  float calculateZeta3() const;

  float calculateSHyp(double camber_angle) const;

  float calculateSVyk(double vertical_load, double slip_ratio) const;
  /**
   * @brief Auxiliary function that returns the sign of a value
   *
   * @param val The value
   * @return int 1 or -1
   */
  static int sign(float val) { return (val > 0) ? 1 : -1; }
};
