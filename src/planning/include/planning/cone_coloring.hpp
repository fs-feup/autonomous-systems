#ifndef SRC_PLANNING_INCLUDE_PLANNING_CONE_COLORING_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_CONE_COLORING_HPP_

#include "utils/cone.hpp"
#include "utils/pose.hpp"
#include "utils/righ_left_enum.hpp"
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>

/**
 * @brief Class responsible for recieving a set of uncolored cones and discovering their color
 * This is done by performing a search that tries to find each side of the track by iteratively
 * selecting the cone with the least cost to be added to each side of the track, according to the
 * cost function defined in the class.
 *  
 */
class ConeColoring {
  private:

     /**
      * @brief angle gain between the last edge and the possible new edge
      * 
      */
    double angle_gain;

     /**
      * @brief distance gain to use for the distance cost
      * 
      */
    double distance_gain;

    /**
      * @brief gain to use for the cost
      * 
      */
    double ncones_gain;

    /**
      * @brief exponent for the sum of distance and angle
      * 
      */
    double exponent1;

    /**
      * @brief exponent for the overall sum
      * 
      */
    double exponent2;

    /**
      * @brief maximum cost to reach a cone
      * 
      */
    double max_cost;

    std::shared_ptr<Cone> virtual_left_cone = nullptr;

    std::shared_ptr<Cone> virtual_right_cone = nullptr;

     /**
      * @brief cost function used to minimize the cost of the path.
      * Based on the idea that, when there are at least two cones on one 
      * side of the track, the next cone is the one that minimizes a combination
      * of the angle formed between consecutiv edges and the distance to the next cone.
      * 
      * Should not be called when current sides of the tracks are empty
      * 
      * @param possible_next_cone possible next cone to be reached
      * @param n number of cones in the input vector
      * @param side reach the cone form the left or right side
      * @return double cost of reaching the cone provided as a parameter
      */
    double cost(const Cone* possible_next_cone, int n, TrackSide side) const;

    /**
      * @brief determine if the specified cone is in the specified side of the track
      * 
      * @param c cone to be checked
      * @param side whether to check if the cone is in the left or right side
      * @return true if the cone is in the side of the track
      * @return false if the cone is not in the side of the track
      */
    bool cone_is_in_side(const Cone *c, TrackSide side) const;

    /**
      * @brief angle between the current last edge and the possible new edge [rad]
      * 
      * @param possible_next_cone possible next cone to be reached
      * @param side reach the cone form the left or right side
      * @param norm variable to place the distance between the last and possible next cone in
      * @return double angle between the current cone and the next cone
      */
    double angle_and_norm(const Cone* possible_next_cone, TrackSide side, double &norm) const ; // [rad]

    /**
     * @brief place the next cone in the path
     * 
     * @param un_visited_cones array of possible cones to be visited (marked with false if not visited yet)
     * @param distance_threshold maximum distance to reach the next cone
     * @param side place the next cone in the left or right side
     * @return true if the cone has been successfully placed
     * @return false if the cone hasn't been placed
     */
    bool place_next_cone(std::shared_ptr<std::vector<std::pair<Cone *, bool>>> visited_cones, double distance_threshold, int n, TrackSide side);

    /**
     * @brief get an estimate to the distance to the left side of the track
     * 
     * @param initial_car_pose 
     * @param side distance to the left or right side
     * @param x coordinate of the point to estimate the distance
     * @param y coordinate of the point to estimate the distance
     * @return double estimate of the distance to the left side track
     * 
     * @note assumming the car is at the initial_car_pose and going straight, the right side of the track can be estimated to be 
     * defined by:  y - (car_y + 2*cos(θ)) - tan(θ)*(x - (car_x  + 2*sen(θ))) = 0
     * and the right side by : y - (car_y - 2*cos(θ)) - tan(θ)*(x - (car_x  - 2*sen(θ))) = 0
     * This also assumes the track width to be 4 meters since the minimum is 3 and the maximum is 5
     */
    double distance_to_side(Pose initial_car_pose, TrackSide side, double x, double y) const;

  public: 
    /**
     * @brief Construct a new Cone Coloring:: Cone Coloring object
     * 
     * @param gain_angle 
     * @param gain_distance 
     * @param gain_ncones 
     * @param exponent_1 
     * @param exponent_2 
     * @param cost_max 
     */
    ConeColoring(double gain_angle, double gain_distance, double gain_ncones,
    double exponent_1, double exponent_2, double cost_max);
    /**
     * @brief color the cones based on the cost to reach them
     * 
     * @param input_cones cones to be colored
     * @param initial_car_pose initial pose of the car
     * @param distance_threshold maximum allowed distance to reach the next cone
     */
    void color_cones(const std::vector<Cone *>& input_cones, Pose initial_car_pose, double distance_threshold);
    /**
     * @brief current cones on the left side
     * 
     */
    std::vector<Cone *> current_left_cones = {};
      /**
       * @brief current cones on the right side
       * 
       */
    std::vector<Cone*> current_right_cones = {};
    /**
     * @brief get the initial cone to start the path from side determined by parameter
     * 
     * @param candidates possible cones to start the path (cones closer than threshold)
     * @param initial_car_pose initial pose of the car
     * @param side reach the cone form the left or right side
     * @return Cone* initial cone to start the path
     */
    Cone* get_initial_cone(const std::vector<Cone*>& candidates, Pose initial_car_pose, TrackSide side);

    /**
      * @brief filter cones that are closer than a certain radius
      * 
      * @param input_cones cones to be filtered
      * @param position position to filter the cones in accordance to
      * @param radius radius to filter the cones
      * @return std::vector<Cone*> filtered cones
      */
    std::vector<Cone*> filter_cones_by_distance(const std::vector<Cone*>& input_cones, Position position, double radius) const;

    /**
     * @brief make a vector of cones and a boolean to determine if the cone has been visited
     * 
     * @param cones cones to be marked as visited [true] or unvisited [false]
     * @return std::vector<std::pair<Cone *, bool> *> vector of cones and a boolean to determine if the cone has been visited
     */ 
    std::shared_ptr<std::vector<std::pair<Cone *, bool>>> make_unvisited_cones(const std::vector<Cone *>& cones) const;

    /**
      * @brief add the best initial cones to the sides of the track
      * 
      * @param input_cones uncolored cones
      * @param initial_car_pose initial pose of the car
      */
    void place_initial_cones(const std::vector<Cone *>& input_cones, Pose initial_car_pose);
};

#endif