#pragma once

#include "base_tire_model.hpp"
#include "common_lib/structures/forces.hpp"
#include "common_lib/structures/velocities.hpp"
#include "common_lib/structures/wheels.hpp"

/*
-------------------------------------------------------------------------------------------
  Only struct to be declared here since it is suposed to be used only for this tire model
-------------------------------------------------------------------------------------------
*/
struct InternalValues {
  double distance_to_CG;
  double camber_angle;
  double epsilong;
  double epsilon = 1e-6;  // safety factor to avoid division by zero
  double dfz;
  double Fz0_prime;
  double Vc;
  double Vc_prime;
  double phi;
  double dpi = 0;  // defaulted to 0 (ignore pressure changes)
  double gamma_star;
  double zeta2;
  double Kya;
  double zeta3;
  double Vcx;
  double zeta1;
  double LMUY_prime;
  double SVyg;
  double alpha_star;
};

// Isto está aqui apenas para compilar, isto será muito provavelemnte o input to tire_forces
struct VehicleModelState {
  common_lib::structures::Velocities velocities;
  common_lib::structures::Forces forces;
  common_lib::structures::Wheels wheels_speed;
  double angular_speed;
  double steering_angle = 0.0;
  double yaw_rate;
};

/*
    Given the proposed logic the tire calculation should:
        if Vx > 3 m/s
            - Compute slip angle (alpha) using:
                - velocity Y, velocity X, yaw rate and distance from cg
            - Compute slip ratio (Kappa) using:
                - velocity X, Tire radius, angular speed
            - Compute pacejka
        if Vx <= 3 m/s - Use lugre
    NOTE: Prevent division by zero on Vx

*/

//------------------------------------------------------------------------------------------------------------------------------------------------
// IMPORTANT: In case of wanting to account for pressure changes, there is currently no
// implementation, we always assume constant tire pressure
//------------------------------------------------------------------------------------------------------------------------------------------------
class MF6_2 : public TireModel {
public:
  explicit MF6_2(const common_lib::car_parameters::CarParameters& car_parameters)
      : TireModel(car_parameters){};

  // Esta definição é precisa para que ele compile, tens de mudar isto para o que for preciso como
  // input e mudar o base model, não sei muito bem como conciliar este com o pacejka_combined_slip,
  // mas o ideal é que ambos usem a mesma interface
  std::pair<double, double> tire_forces(double slip_angle, double slip_ratio,
                                        double vertical_load) const override;

private:
  InternalValues internal_vals;
  VehicleModelState vehicle_model_state;

  // /**
  //  * @brief Calculates slip angle using velocity and yawRate
  //  *        Normalizes v_x to prevent division by 0
  //  * @return Corresponds to arctan((v_y + yawR*distance)/v_x)
  //  */
  // float calculateSlipAngle() const;

  // /**
  //  * @brief Calculates the slip ration using velocity and angular speed
  //  *         Normalizes v_x to prevent division by 0
  //  * @return Corresponds to ((effectiveTireRadius * AngularSpeed) - v_x) / v_x
  //  */
  // float calculateSlipRatio() const;

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
   * @brief Auxiliary function that computes all intermediate values needed
   * 
   * @param slip_angle  Slip angle  
   * @param slip_ratio Slip ratio 
   * @param vertical_load Fz
   * @return true Successful calculation of intermediate values
   * @return false 
   */
  bool calculateTireState(float slip_angle, float slip_ratio, double vertical_load);

  /**
   * @brief Calculates the D parameter for the Fy calculation using pacejka MF (Peak)
   *
   * @return float The value of Dy
   */
  float calculateDy(double vertical_load) const;

  float calculateDx(double vertical_load) const;

  /**
   * @brief Calculates the C component for the pure longitudinal slip (4.E11)
   * 
   * @return float value of Cx
   */
  float calculateCx(double vertical_load) const;

  float calculateCy(double vertical_load) const;

  float calculateBx(float Dx, float Cx, double vertical_load) const;

  float calculateBy(float Dy, float Cy) const;

  /**
   * @brief Calculates the E component for the longitudinal slip 
   * 
   * @param shifted_slip_ratio The already shited slip ratio (kappax) 
   * @return float Value of Ex
   */
  float calculateEx(double shifted_slip_ratio) const;

  float calculateEy(double slip_angle, double vertical_load) const;

  float calculateSVx(double vertical_load) const;

  float calculateSVy(double vertical_load) const;
/**
 * @brief Calculates the shift for the slip ratio calculation
 * 
 * @return float Shift
 */
  float calculateSHx() const;

  /**
   * @brief Calculates Fx using combined slip
   * 
   * @param Fx0 Fx calculated using pure slip
   * @return float Fx using combined slip
   */
  float calculateCombinedLongitudinal(double Fx0) const;


  /**
   * @brief Calculates Fy using combined slip
   * 
   * @param Fy0  Fy calculated using pure slip  
   * @param shifted_slip_ratio Slip ratio already shifted (kappax)  
   * @param slip_ratio Slip ratio (kappa)
   * @param vertical_load Fz
   * @return float Fy using combined slip
   */
  float calculateCombinedLateral(double Fy0 , double shifted_slip_ratio, double slip_ratio , double vertical_load) const;

  /**
   * @brief Auxiliary function that returns the sign of a value
   * 
   * @param val The value
   * @return int 1 or -1
   */
  static int sign(float val) { return (val > 0) ? 1 : -1; }
};
