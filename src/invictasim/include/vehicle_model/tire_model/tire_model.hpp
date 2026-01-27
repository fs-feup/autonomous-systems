#pragma once
#include "../vd_helpers.hpp"

struct TireInput {
  /* Values*/
  velocity v;
  float yawR;
  float angSpeed;
};

// Could add combined slip if needed
struct TireOutput {
  double Fx;
  double Fy;
};

// maybe change to the actual names?
struct PacejkaParameters {
  float B = 1.0;
  float C = 1.0;
  float D = 1.0;
  float E = 1.0;
};

/*
    Struct containing the tire radius, camber...
*/
struct TireParameters {
  float effectiveTireR = 1.0;
  float distanceToCG =
      1.0;  // MUST BE POSITIVE IF AHEAD OF CG AND NEGATIVE IF BEHIND OTHERWISE PACEJKA WILL BREAK
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

class TireModel {
public:
  explicit TireModel(float effective_radius, float distance_to_cg, PacejkaParameters pacejka);

  TireModel();
  /**
   * @brief Calculates the resulting forces from tire input
   *
   * @param input velocity, yawRate and angular speed
   * @return TireOutput
   */
  TireOutput calculateForces(const TireInput& input)
      const;  // Maybe TireOutput should be passed and returned by reference?

  /**
   * @brief Calculates slip angle using velocity and yawRate
   *        Normalizes v_x to prevent division by 0
   * @return Corresponds to arctan((v_y + yawR*distance)/v_x)
   */
  float calculateSlipAngle(float v_x, float v_y, float yawR) const;

  /**
   * @brief Calculates the slip ration using velocity and angular speed
   *         Normalizes v_x to prevent division by 0
   * @return Corresponds to ((effectiveTireRadius * AngularSpeed) - v_x) / v_x
   */
  float calculateSlipRatio(float angularSpeed, float v_x) const;

  /**
   * @brief Calculates tire force using the respective slip using pacejka
   *
   * @param slip SlipAngle to calculate lateral force and SlipRatio for longitudinal force
   * @return float Either Fy or Fx depending on chosen slip
   */
  float calculatePacejka(float slip) const;

  float get_tire_effective_radius() const;

private:
  /*Values*/
  TireParameters tireParameters_;
  PacejkaParameters pacejka;
};
