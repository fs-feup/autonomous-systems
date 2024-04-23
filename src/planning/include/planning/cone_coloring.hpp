#ifndef SRC_PLANNING_INCLUDE_PLANNING_CONE_COLORING_HPP_
#define SRC_PLANNING_INCLUDE_PLANNING_CONE_COLORING_HPP_

#include "utils/cone.hpp"
#include "loc_map/data_structures.hpp"

class ConeColoring {
 public:
    /**
     * @brief Construct a new Cone Coloring object
     * 
     */
  ConeColoring();

   /**
    * @brief angle gain between the last edge and the possible new edge
    * 
    */
  double angle_gain = 0.5;

   /**
    * @brief distance gain to use for the distance cost
    * 
    */
  double distance_gain = 0.5;

  /**
    * @brief gain to use for the cost
    * 
    */
  double ncones_gain = 0.5;

   /**
    * @brief cost function used to minimize the cost of the path.
    * Should not be called when current sides of the tracks are empty
    * 
    * @param possible_next_cone possible next cone to be reached
    * @param n number of cones in the input vector
    * @param side reach the cone form the left [true] or right [false] side
    * @return double cost of reaching the cone provided as a parameter
    */
  double cost(Cone* possible_next_cone, int n, bool side);

  /**
    * @brief determine the initial cone to start the track side
    * 
    * @param input_cones uncolored cones
    * @param side cost to reach the cone form the left [true] or right [false] side
    * @return Cone initial cone to start the path
    */
  void place_initial_cones(std::vector<Cone *> input_cones, Pose initial_car_pose);

  /**
    * @brief angle between the current last edge and the possible new edge [rad]
    * 
    * @param possible_next_cone possible next cone to be reached
    * @param side reach the cone form the left [true] or right [false] side
    * @param norm variable to place the distance between the last and possible next cone in
    * @return double angle between the current cone and the next cone
    */
  double angle_and_norm(Cone *possible_next_cone, bool side, double &norm); // [rad]

  /**
    * @brief filter cones that are closer than a certain radius
    * 
    * @param input_cones cones to be filtered
    * @param x x coordinate of the point to filter the cones
    * @param y y coordinate of the point to filter the cones
    * @param radius radius to filter the cones
    * @return std::vector<Cone*> filtered cones
    */
  std::vector<Cone *> filterCones(std::vector<Cone *> input_cones, double x, double y, double radius);

  /**
    * @brief squared distance between two cones
    * 
    * @param c1 first cone
    * @param c2 second cone
    * @return double squared distance between the two cones
    */
  double squaredDistance(Cone *c1, Cone *c2);

  /**
   * @brief place the next cone in the path
   * 
   * @param un_visited_cones array of possible cones to be visited (marked with false if not visited yet)
   * @param distance_threshold maximum distance to reach the next cone
   * @param cost_threshold maximum allowed cost to reach the next cone
   * @param side place the next cone in the left [true] or right [false] side
   * @return true if the cone has been successfully placed
   * @return false if the cone hasn't been placed
   */
  bool placeNextCone(std::vector<std::pair<Cone *, bool>> &visited_cones, double distance_threshold, double cost_threshold, int n, bool side);
  
  /**
   * @brief color the cones based on the cost to reach them
   * 
   * @param input_cones cones to be colored
   * @param initial_car_pose initial pose of the car
   * @param cost_threshold maximum allowed cost to reach the next cone
   * @param distance_threshold maximum allowed distance to reach the next cone
   */
  void colorCones(std::vector<Cone *> input_cones, Pose initial_car_pose, double cost_threshold, double distance_threshold);

  /**
   * @brief get an estimate to the distance to the left side of the track
   * 
   * @param initial_car_pose 
   * @param side distance to the left [true] or right [false] side
   * @param x coordinate of the point to estimate the distance
   * @param y coordinate of the point to estimate the distance
   * @return double estimate of the distance to the left side track
   * 
   * @note assumming the car is at the initial_car_pose and going straight, the right side of the track can be estimated to be 
   * defined by:  y - (car_y + 2*cos(θ)) - tan(θ)*(x - (car_x  + 2*sen(θ))) = 0
   * and the right side by : y - (car_y - 2*cos(θ)) - tan(θ)*(x - (car_x  - 2*sen(θ))) = 0
   * This also assumes the track width to be 4 meters since the minimum is 3 and the maximum is 5
   */
  double distance_to_side(Pose initial_car_pose, bool side, double x, double y);

  /**
   * @brief get the initial cone to start the path from side determined by parameter
   * 
   * @param candidates possible cones to start the path (cones closer than threshold)
   * @param initial_car_pose initial pose of the car
   * @param side reach the cone form the left [true] or right [false] side
   * @return Cone* initial cone to start the path
   */
  Cone* getInitialCone(std::vector<Cone *> candidates, Pose initial_car_pose, bool side);

  /**
   * @brief current cones on the left side
   * 
   */
  std::vector<Cone *> current_left_cones;

    /**
     * @brief current cones on the right side
     * 
     */
  std::vector<Cone *> current_right_cones;
};

#endif