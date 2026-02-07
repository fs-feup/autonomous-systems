#pragma once

#include "common_lib/car_parameters/car_parameters.hpp"

// O TIRE INPUT TEM DE SER MELHOR DEFINIDO,
// O PACEJKA QUE JÁ HAVIOA USA UM INPUT MT DIFERENTE DO BOMBADO,
// É PRECISO DEFINIR UMA INTERFACE QUE TODOS USAM COMO INPUT PARA O TIRE_FORCES

// POSSIVELMENTE UM STRUCT TireInput COM TODOS OS INPUTS DE TODOS OS MODELOS E CADA USAM USA OS
// SEUS, POR ENQUANTO NÃO MEXI POIS NÃO O QUE FALTA NO BOMBADO

/**
 * @brief Class used to model tires. Currently used to model tire forces based on slip.
 *
 */
class TireModel {
protected:
  std::shared_ptr<common_lib::car_parameters::CarParameters> car_parameters_;

public:
  TireModel(const common_lib::car_parameters::CarParameters& car_parameters)
      : car_parameters_(
            std::make_shared<common_lib::car_parameters::CarParameters>(car_parameters)) {}
  /**
   * @brief Calculate the forces acting in a tire based on the tire characteristics and dynamic
   * state.
   *
   * @param slip_angle Slip angle of the tire in radians
   * @param slip_ratio Slip ratio of the tire
   * @param load Load on the tire in Newtons
   * @return std::pair<double, double> Longitudinal and lateral forces
   */
  virtual std::pair<double, double> tire_forces(double slip_angle, double slip_ratio,
                                                double vertical_load) const = 0;
};
