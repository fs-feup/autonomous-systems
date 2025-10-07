#pragma once

#include <functional>

#include "common_lib/structures/cone.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_2.h>
#include <iostream>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = K::Point_2;

namespace common_lib::structures {
/**
 * @brief MidPoint struct represents a potential path point with connections
 */
struct Midpoint {
    Point point;
    std::vector<std::shared_ptr<Midpoint>> close_points; 
    std::shared_ptr<Cone> cone1;
    std::shared_ptr<Cone> cone2;
    bool valid = true;

    Midpoint() = default; 
    Midpoint(const Point& p,
        std::shared_ptr<Cone> c1,
        std::shared_ptr<Cone> c2);

    friend bool operator==(const Midpoint& a, const Midpoint& b) {
        return a.point == b.point;
    }
};

}  // namespace common_lib::structures

/**
 * @brief Hash functions for Points and MidPoints
 */
namespace std {
  template <>
  struct hash<Point> {
    size_t operator()(const Point& p) const noexcept {
      auto hx = std::hash<double>()(p.x());
      auto hy = std::hash<double>()(p.y());
      return hx ^ (hy << 1); 
    }
  };

  template <>
  struct hash<common_lib::structures::Midpoint> {
    size_t operator()(const common_lib::structures::Midpoint& mid) const noexcept {
      return std::hash<Point>()(mid.point);
    }
  };

}  // namespace std