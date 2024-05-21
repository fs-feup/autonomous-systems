#pragma once

#include "common_lib/structures/position.hpp"

namespace common_lib::structures {

    struct PathPoint {
        Position position;
        double ideal_velocity = 1.0;
        PathPoint() = default;
        PathPoint(Position position, double certainty);
        PathPoint(double x, double y, double certainty);
        PathPoint(PathPoint const &path_point) = default;
        double getX() const;
        double getY() const;
        double getV() const;
    };
}   // namespace common_lib::structures