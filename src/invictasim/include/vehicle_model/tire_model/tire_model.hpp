#pragma once
#include "../vd_helpers.hpp"

struct TireInput {
  Velocity v;
  float yawR;
  float angSpeed;
};

/*
-------------------------------------------------------------------------------------------
  Only struct to be declared here since it is suposed to be used only for the tire model
-------------------------------------------------------------------------------------------
*/
struct InternalValues {
  double distance_to_CG;
  double camber_angle;
  double epsilong;
  double epsilon = 1e-6;  // safety factor to avoid division by zero
  double dfz;
  double Fz0_prime;
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
};

// Could add combined slip if needed
struct TireOutput {
  double Fx;
  double Fy;
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
class TireModel {
public:
  explicit TireModel(float camber, float dist);

  /**
   * @brief Calculates the resulting forces from tire input
   *
   * @param input velocity, yawRate and angular speed
   * @return TireOutput
   */
  TireOutput calculateForces()
      const;  // Maybe TireOutput should be passed and returned by reference?

  /**
   * @brief Calculates slip angle using velocity and yawRate
   *        Normalizes v_x to prevent division by 0
   * @return Corresponds to arctan((v_y + yawR*distance)/v_x)
   */
  float calculateSlipAngle() const;

  /**
   * @brief Calculates the slip ration using velocity and angular speed
   *         Normalizes v_x to prevent division by 0
   * @return Corresponds to ((effectiveTireRadius * AngularSpeed) - v_x) / v_x
   */
  float calculateSlipRatio() const;

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
  float calculatePacejka(float B, float C, float D, float E, float slip, float SV) const;

  // REMOVE
  float get_tire_effective_radius() const;

  bool calculateTireState(float slip_angle, float slip_ratio);

  /**
   * @brief Calculates the D parameter for the Fy calculation using pacejka MF (Peak)
   *
   * @return float The value of Dy
   */
  float calculateDy() const;

  float calculateDx() const;

  float calculateCx() const;

  float calculateCy() const;

  float calculateBx(float Dx, float Cx) const;

  float calculateBy(float Dy, float Cy) const;

  float calculateEx(double slip_ratio) const;

  float calculateEy(double slip_angle) const;

  float calculateSVx() const;

  float calculateSVy() const;

private:
  /*Values*/
  InternalValues internal_vals;
  VehicleModelState vehicle_model_state;
  VehicleModelParams vehicle_model_params;
};
