#include "planning/cone_coloring.hpp"
#include <iostream>
#include <memory>
#include <algorithm>

ConeColoring::ConeColoring(double gain_angle, double gain_distance, double gain_ncones,
 double exponent_1, double exponent_2, double cost_max ) : angle_gain(gain_angle),
 distance_gain(gain_distance), ncones_gain(gain_ncones), exponent1(exponent_1),
 exponent2(exponent_2), max_cost(cost_max) {}

double ConeColoring::angle_and_norm(const Cone* possible_next_cone, TrackSide side, double &norm) const {
    std::vector<Cone*> current_cones = (bool)side ? current_left_cones : current_right_cones;
    auto n = (int)current_cones.size();
    double x_last = current_cones[n-1] -> getX() - current_cones[n-2] -> getX();
    double y_last = current_cones[n-1] -> getY() - current_cones[n-2] -> getY();
    double norm2 = sqrt(pow(x_last,2) + pow(y_last,2));
    double x_next = possible_next_cone -> getX() - current_cones[n-1] -> getX();
    double y_next = possible_next_cone -> getY() - current_cones[n-1] -> getY();
    norm = sqrt(pow(x_next,2) + pow(y_next,2));
    double dot_product = x_last*x_next + y_last*y_next;
    return acos(dot_product/(norm*norm2));
}

bool ConeColoring::cone_is_in_side(const Cone *c, TrackSide side) const {
    std::vector<Cone*> current_cones = (bool)side ? current_left_cones : current_right_cones;
    return std::any_of(current_cones.begin(), current_cones.end(), [c](const Cone* cone) {
        return cone->getId() == c->getId();
    });
}

double ConeColoring::distance_to_side(Pose initial_car_pose, TrackSide side, double x, double y) const {
    if ((bool)side) return fabs(y - (initial_car_pose.position.y + 2*cos(initial_car_pose.orientation)) -
     tan(initial_car_pose.orientation)*( x - ( initial_car_pose.position.x - 2*sin(initial_car_pose.orientation))));
    else return fabs(y - (initial_car_pose.position.y - 2*cos(initial_car_pose.orientation)) -
     tan(initial_car_pose.orientation)*( x - ( initial_car_pose.position.x + 2*sin(initial_car_pose.orientation))));
}

Cone* ConeColoring::getInitialCone(const std::vector<Cone *>& candidates, Pose initial_car_pose, TrackSide side) {
    double minimum_cost = MAXFLOAT;
    Cone* initial_cone = nullptr;
    for (Cone* c : candidates) {
        double distance_to_car = sqrt(pow(initial_car_pose.position.x - c -> getX(), 2) + pow(initial_car_pose.position.y - c -> getY(), 2));
        double cost = distance_to_car * distance_to_side(initial_car_pose, side, c -> getX(), c -> getY());
        if (cost < minimum_cost) {
            minimum_cost = cost;
            initial_cone = std::move(c);
        }
    }
    return initial_cone;
}

std::vector<Cone*> ConeColoring::filter_cones_by_distance(const std::vector<Cone *>& input_cones, Position position, double radius) const {
    std::vector<Cone*> filtered_cones;
    for (Cone* c : input_cones) {
        double squared_distance = pow(position.x - c -> getX(), 2) + pow(position.y - c -> getY(), 2);
        if (squared_distance <= radius*radius) filtered_cones.push_back(c);
    }
    return filtered_cones;
}

void ConeColoring::place_initial_cones(const std::vector<Cone *>& input_cones, Pose initial_car_pose) {
    // filter cones that are farther away than 5 meters
    std::vector<Cone*> candidates = filter_cones_by_distance(input_cones, initial_car_pose.position, 5);

    // get the initial cone for the left side
    Cone* initial_cone_left = getInitialCone(candidates, initial_car_pose, TrackSide::left);

    // get the initial cone for the right side 
    Cone* initial_cone_right = getInitialCone(candidates, initial_car_pose, TrackSide::right);

    // Create virtual cones to be added at the beginning of the vector. They must be dismissed later.
    auto virtual_left_cone = std::make_shared<Cone>(-2, initial_cone_left->getX() - 2*(float)cos(initial_car_pose.orientation), initial_cone_left -> getY() - 2*(float)sin(initial_car_pose.orientation));
    auto virtual_right_cone = std::make_shared<Cone>(-1, initial_cone_right->getX() - 2*(float)cos(initial_car_pose.orientation), initial_cone_right->getY() - 2*(float)sin(initial_car_pose.orientation));

    current_left_cones.push_back(virtual_left_cone.get());
    current_right_cones.push_back(virtual_right_cone.get());
    current_left_cones.push_back(initial_cone_left);
    current_right_cones.push_back(initial_cone_right);
}

double ConeColoring::cost(const Cone* possible_next_cone, int n, TrackSide side) const {
    const std::vector<Cone*>& current_cones = (bool)side ? current_left_cones : current_right_cones;
    double norm;
    double angle = angle_and_norm(possible_next_cone, side, norm);
    double cost = pow(pow(distance_gain*norm + angle_gain*angle, exponent1) + ncones_gain*(double)current_cones.size()/(float)n, exponent2);
    return cost;
}

std::vector<std::pair<Cone *, bool> *> ConeColoring::makeUnvisitedCones(const std::vector<Cone *>& cones) const {
    std::vector<std::pair<Cone *, bool> *> conePairs;
    for (Cone *cone : cones) {
        auto conePair = std::make_shared<std::pair<Cone *, bool>>(cone, cone_is_in_side(cone, TrackSide::left) || cone_is_in_side(cone, TrackSide::right));
        conePairs.push_back(conePair.get());
    }
    return conePairs;
}

bool ConeColoring::placeNextCone(std::vector<std::pair<Cone *, bool> *> &visited_cones, double distance_threshold, int n, TrackSide side) {
    
    double minimum_cost = MAXFLOAT;
    
    std::vector<Cone*> &current_cones = (bool)side ? current_left_cones : current_right_cones;

    const Cone *last_cone = current_cones.back();

    Cone *next_cone = visited_cones[0]->first;

    std::pair<Cone *, bool> *best_pair = visited_cones[0];

    for (std::pair<Cone *, bool>* p : visited_cones) {
        if (p -> second) continue; // skip visited cones
        double cost = ConeColoring::cost(p -> first, n, side);
        if (cost < minimum_cost && last_cone->squaredDistanceTo(p ->first) < distance_threshold*distance_threshold && cost < max_cost) {
            minimum_cost = cost;
            next_cone = p -> first;
            best_pair = p;
        }
    }
    if (minimum_cost == MAXFLOAT) return false;
    current_cones.push_back(next_cone);
    best_pair -> second = true;
    return true;
}

void ConeColoring::colorCones(const std::vector<Cone *>& input_cones, Pose initial_car_pose, double distance_threshold) {
    place_initial_cones(input_cones, initial_car_pose);
    std::vector<std::pair<Cone *, bool> *> visited_cones = makeUnvisitedCones(input_cones);

    while (placeNextCone(visited_cones, distance_threshold, (int)input_cones.size(), TrackSide::left));
    while (placeNextCone(visited_cones, distance_threshold, (int)input_cones.size(), TrackSide::right));
}

