#ifndef SRC_PLANNING_INCLUDE_PLANNING_OUTLIERS_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_OUTLIERS_HPP_

#include "common_lib/competition_logic/color.hpp"
#include "common_lib/structures/cone.hpp"
#include "config/outliers_config.hpp"
#include "utils/splines.hpp"

using Cone = common_lib::structures::Cone;
using Color = common_lib::competition_logic::Color;

/**
 * @brief Class to deal with outliers
 *
 */
class Outliers {
private:
  /**
   * @brief configuration of the outliers removal algorithm
   *
   */
  OutliersConfig config_;

public:
  /**
   * @brief Construct a new default Outliers object
   *
   */
  Outliers() = default;
  /**
   * @brief Construct a new Outliers object with a given configuration
   *
   */
  explicit Outliers(const OutliersConfig& config) : config_(config) {}
  /**
   * @brief function to remove outliers from a set of cones
   *
   * @param cones input pair of vectors of ordered cones
   * @return std::pair<std::vector<Cone>, std::vector<Cone>> pair of vectors of ordered cones in
   * the same order as recieved, but all cones are shifted to the general spline approximation,
   * dealing with outliers
   */
  std::pair<std::vector<Cone>, std::vector<Cone>> approximate_cones_with_spline(
      std::pair<std::vector<Cone>, std::vector<Cone>>& cones) const;
};

#endif  // SRC_PLANNING_INCLUDE_PLANNING_OUTLIERS_HPP_