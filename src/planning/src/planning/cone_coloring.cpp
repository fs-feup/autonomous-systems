#include "planning/cone_coloring.hpp"
#include <queue>
using namespace std;

void ConeColoring::remove_duplicates(std::vector<Cone>& cones) const {
  std::vector<Cone> clustered_cones;
  clustered_cones.reserve(cones.size());

  for (const auto& cone : cones) {
    bool is_duplicate = false;

    // Check against each cone in the clustered_cones list
    for (const auto& clustered_cone : clustered_cones) {
      if (cone.position.euclidean_distance(clustered_cone.position) <
          this->config_.same_cone_distance_threshold_) {
        is_duplicate = true;
        break;
      }
    }

    // If not a duplicate, add to the clustered list
    if (!is_duplicate) {
      clustered_cones.push_back(cone);
    }
  }

  // Update original cones vector with clustered cones
  cones = clustered_cones;
}

std::vector<Cone> ConeColoring::filter_previously_colored_cones(const std::vector<Cone>& cones) {
  std::vector<Cone> uncolored_cones;
  bool seen;
  for (const auto& cone : cones) {
    seen = false;
    if (!seen) {
      for (auto& colored_cone : this->colored_blue_cones_) {
        if (cone.position.euclidean_distance(colored_cone.position) <
            this->config_.same_cone_distance_threshold_) {
          seen = true;
          colored_cone.position.x = cone.position.x;
          colored_cone.position.y = cone.position.y;
          break;
        }
      }
    }

    if (!seen) {
      for (auto& colored_cone : this->colored_yellow_cones_) {
        if (cone.position.euclidean_distance(colored_cone.position) <
            this->config_.same_cone_distance_threshold_) {
          seen = true;
          colored_cone.position.x = cone.position.x;
          colored_cone.position.y = cone.position.y;
          break;
        }
      }
    }
    if (!seen) {
      uncolored_cones.push_back(cone);
    }
  }
  return uncolored_cones;
}

Position ConeColoring::expected_initial_cone_position(const Pose& car_pose,
                                                      const TrackSide& track_side) const {
  constexpr double distance_to_car = 1.5;
  if (track_side == TrackSide::LEFT) {
    return Position{car_pose.position.x - distance_to_car * sin(car_pose.orientation),
                    car_pose.position.y + distance_to_car * cos(car_pose.orientation)};
  } else if (track_side == TrackSide::RIGHT) {
    return Position{car_pose.position.x + distance_to_car * sin(car_pose.orientation),
                    car_pose.position.y - distance_to_car * cos(car_pose.orientation)};
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ConeColoring"), "Invalid track side");
    return Position{0, 0};
  }
}

Cone ConeColoring::find_initial_cone(const std::unordered_set<Cone, std::hash<Cone>>& cones,
                                     const Pose& car_pose, const TrackSide track_side) const {
  Position expected_cone_position = expected_initial_cone_position(car_pose, track_side);
  Cone initial_cone = *std::min_element(
      cones.begin(), cones.end(), [&expected_cone_position](const Cone& cone1, const Cone& cone2) {
        return cone1.position.euclidean_distance(expected_cone_position) <
               cone2.position.euclidean_distance(expected_cone_position);
      });
  return initial_cone;
}

void ConeColoring::place_initial_cones(std::unordered_set<Cone, std::hash<Cone>>& uncolored_cones,
                                       const Pose& car_pose, int& n_colored_cones) {
  Position expected_blue_cone_position = expected_initial_cone_position(car_pose, TrackSide::LEFT);
  Position expected_yellow_cone_position =
      expected_initial_cone_position(car_pose, TrackSide::RIGHT);

  Cone initial_cone_left =
      *std::min_element(uncolored_cones.begin(), uncolored_cones.end(),
                        [&expected_blue_cone_position](const Cone& cone1, const Cone& cone2) {
                          return cone1.position.euclidean_distance(expected_blue_cone_position) <
                                 cone2.position.euclidean_distance(expected_blue_cone_position);
                        });
  Cone initial_cone_right =
      *std::min_element(uncolored_cones.begin(), uncolored_cones.end(),
                        [&expected_yellow_cone_position](const Cone& cone1, const Cone& cone2) {
                          return cone1.position.euclidean_distance(expected_yellow_cone_position) <
                                 cone2.position.euclidean_distance(expected_yellow_cone_position);
                        });

  if (initial_cone_left.position.euclidean_distance(expected_blue_cone_position) >
      initial_cone_right.position.euclidean_distance(expected_yellow_cone_position)) {
    expected_blue_cone_position = {expected_blue_cone_position.x + initial_cone_right.position.x -
                                       expected_yellow_cone_position.x,
                                   expected_blue_cone_position.y + initial_cone_right.position.y -
                                       expected_yellow_cone_position.y};
    uncolored_cones.erase(initial_cone_right);
    colored_yellow_cones_.push_back(initial_cone_right);
    n_colored_cones++;
    initial_cone_left =
        *std::min_element(uncolored_cones.begin(), uncolored_cones.end(),
                          [&expected_blue_cone_position](const Cone& cone1, const Cone& cone2) {
                            return cone1.position.euclidean_distance(expected_blue_cone_position) <
                                   cone2.position.euclidean_distance(expected_blue_cone_position);
                          });
    uncolored_cones.erase(initial_cone_left);
    colored_blue_cones_.push_back(initial_cone_left);
    n_colored_cones++;
  } else {
    expected_yellow_cone_position = {
        expected_yellow_cone_position.x + initial_cone_left.position.x -
            expected_blue_cone_position.x,
        expected_yellow_cone_position.y + initial_cone_left.position.y -
            expected_blue_cone_position.y};
    uncolored_cones.erase(initial_cone_left);
    colored_blue_cones_.push_back(initial_cone_left);
    n_colored_cones++;
    initial_cone_right = *std::min_element(
        uncolored_cones.begin(), uncolored_cones.end(),
        [&expected_yellow_cone_position](const Cone& cone1, const Cone& cone2) {
          return cone1.position.euclidean_distance(expected_yellow_cone_position) <
                 cone2.position.euclidean_distance(expected_yellow_cone_position);
        });
    uncolored_cones.erase(initial_cone_right);
    colored_yellow_cones_.push_back(initial_cone_right);
    n_colored_cones++;
  }
  place_second_cones(uncolored_cones, car_pose, n_colored_cones);
}

Cone ConeColoring::virtual_cone_from_initial_cone(const Cone& initial_cone,
                                                  const Pose& car_pose) const {
  constexpr double distance_to_virtual_cone = 2.0;
  return Cone{
      Position{initial_cone.position.x - distance_to_virtual_cone * cos(car_pose.orientation),
               initial_cone.position.y - distance_to_virtual_cone * sin(car_pose.orientation)},
      initial_cone.color, 1.0};
}

void ConeColoring::place_second_cones(std::unordered_set<Cone, std::hash<Cone>>& uncolored_cones,
                                      const Pose& car_pose, int& n_colored_cones) {
  Cone first_left_cone = colored_blue_cones_.front();
  Cone first_right_cone = colored_yellow_cones_.front();

  double car_orientation = car_pose.orientation;
  double min_distance = std::numeric_limits<double>::max();

  Cone second_left_cone;
  for (const auto& cone : uncolored_cones) {
    double segment_orientation = atan2(cone.position.y - first_left_cone.position.y,
                                       cone.position.x - first_left_cone.position.x);
    segment_orientation = std::fmod(segment_orientation + 2.0 * M_PI, 2.0 * M_PI);
    double orientation_difference = std::fabs(car_orientation - segment_orientation);
    if (orientation_difference > 2.0 * M_PI / 5.0 && orientation_difference < 8.0 * M_PI / 5.0) {
      continue;
    }
    double distance = cone.position.euclidean_distance(first_left_cone.position);
    if (distance < min_distance) {
      min_distance = distance;
      second_left_cone = cone;
    }
  }
  if (min_distance < std::numeric_limits<double>::max()) {
    colored_blue_cones_.push_back(second_left_cone);
    uncolored_cones.erase(second_left_cone);
    n_colored_cones++;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ConeColoring"), "Could not find second left cone");
  }

  min_distance = std::numeric_limits<double>::max();
  Cone second_right_cone;
  for (const auto& cone : uncolored_cones) {
    double segment_orientation = atan2(cone.position.y - first_right_cone.position.y,
                                       cone.position.x - first_right_cone.position.x);
    segment_orientation = std::fmod(segment_orientation + 2.0 * M_PI, 2.0 * M_PI);
    double orientation_difference = std::fabs(car_orientation - segment_orientation);
    if (orientation_difference > 2.0 * M_PI / 5.0 && orientation_difference < 8.0 * M_PI / 5.0) {
      continue;
    }
    double distance = cone.position.euclidean_distance(first_right_cone.position);
    if (distance < min_distance) {
      min_distance = distance;
      second_right_cone = cone;
    }
  }
  if (min_distance < std::numeric_limits<double>::max()) {
    colored_yellow_cones_.push_back(second_right_cone);
    uncolored_cones.erase(second_right_cone);
    n_colored_cones++;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ConeColoring"), "Could not find second right cone");
  }
}

double get_curvature(Cone point1, Cone point2, Cone point3) {
  double x1 = point1.position.x;
  double y1 = point1.position.y;
  double x2 = point2.position.x;
  double y2 = point2.position.y;
  double x3 = point3.position.x;
  double y3 = point3.position.y;

  Cone mid1 = Cone((x1 + x2) / 2, (y1 + y2) / 2, "unknown", 1);
  Cone mid2 = Cone((x2 + x3) / 2, (y2 + y3) / 2, "unknown", 1);

  double slope1 = (x2 != x1) ? ((y2 - y1) / (x2 - x1)) : 1000000;
  double slope2 = (x3 != x2) ? ((y3 - y2) / (x3 - x2)) : 1000000;
  double slope1_perpendicular = (slope1 != 0) ? -1 / slope1 : 1000000;
  double slope2_perpendicular = (slope2 != 0) ? -1 / slope2 : 1000000;

  if (slope1_perpendicular == slope2_perpendicular) return 0;

  double center_x = (slope1_perpendicular * mid1.position.x -
                     slope2_perpendicular * mid2.position.x + mid2.position.y - mid1.position.y) /
                    (slope1_perpendicular - slope2_perpendicular);
  double center_y = slope1_perpendicular * (center_x - mid1.position.x) + mid1.position.y;
  double radius = sqrt(pow(center_x - x2, 2) + pow(center_y - y2, 2));

  return 1 / radius;
}

double ConeColoring::calculate_cost(const Cone& next_cone, const Cone& last_cone,
                                    const Cone& second_last_cone,
                                    const TwoDVector& previous_to_last_vector,
                                    const double& colored_to_input_cones_ratio) const {
  AngleAndNorms angle_and_norms = common_lib::maths::angle_and_norms(
      previous_to_last_vector, Position{next_cone.position.x - last_cone.position.x,
                                        next_cone.position.y - last_cone.position.y});
  double distance = angle_and_norms.norm2_;
  double angle = angle_and_norms.angle_;
  double curvature = get_curvature(next_cone, last_cone, second_last_cone);
  double curvature_weight = 5;
  double cost = this->config_.distance_weight_ * pow(distance, this->config_.distance_exponent_) +
                this->config_.angle_weight_ * pow(angle, this->config_.angle_exponent_) +
                this->config_.ncones_weight_ * colored_to_input_cones_ratio +
                curvature_weight * curvature;
  return cost;
}

void ConeColoring::remove_too_close_cones() {
  for (int i = static_cast<int>(this->colored_blue_cones_.size()) - 1; i > 0; i--) {
    if (this->colored_blue_cones_[i - 1].position.euclidean_distance(
            this->colored_blue_cones_[i].position) < this->config_.same_cone_distance_threshold_) {
      this->colored_blue_cones_.erase(this->colored_blue_cones_.begin() + i);
    }
  }
  for (int i = static_cast<int>(this->colored_yellow_cones_.size()) - 1; i > 0; i--) {
    if (this->colored_yellow_cones_[i - 1].position.euclidean_distance(
            this->colored_yellow_cones_[i].position) <
        this->config_.same_cone_distance_threshold_) {
      this->colored_yellow_cones_.erase(this->colored_yellow_cones_.begin() + i);
    }
  }
  // Check and remove cones that are too close between blue and yellow cones
  for (int i = static_cast<int>(this->colored_blue_cones_.size()) - 1; i > 1; i--) {
    for (int j = static_cast<int>(this->colored_yellow_cones_.size()) - 1; j > 1; j--) {
      double distance = this->colored_blue_cones_[i].position.euclidean_distance(
          this->colored_yellow_cones_[j].position);
      if (distance < this->config_.same_cone_distance_threshold_) {
        Cone last_cone = this->colored_blue_cones_[i - 1];
        Cone previous_cone = this->colored_blue_cones_[i - 2];
        TwoDVector last_vector = {last_cone.position.x - previous_cone.position.x,
                                  last_cone.position.y - previous_cone.position.y};
        double blue_cost =
            calculate_cost(this->colored_blue_cones_[i], last_cone, previous_cone, last_vector, 0);
        last_cone = this->colored_yellow_cones_[j - 1];
        previous_cone = this->colored_yellow_cones_[j - 2];
        last_vector = {last_cone.position.x - previous_cone.position.x,
                       last_cone.position.y - previous_cone.position.y};
        double yellow_cost = calculate_cost(this->colored_yellow_cones_[j], last_cone,
                                            previous_cone, last_vector, 0);

        if (blue_cost > yellow_cost) {
          this->colored_blue_cones_.erase(this->colored_blue_cones_.begin() + i);
          break;  // Exit inner loop as this blue cone has been removed
        } else {
          this->colored_yellow_cones_.erase(this->colored_yellow_cones_.begin() + j);
        }
      }
    }
  }
}

bool ConeColoring::try_to_color_next_cone(
    std::unordered_set<Cone, std::hash<Cone>>& uncolored_cones, std::vector<Cone>& colored_cones,
    int& n_colored_cones, const int n_input_cones) {
  double min_cost = std::numeric_limits<double>::max();
  Cone cheapest_cone;
  const Cone last_cone = colored_cones.back();
  const Cone second_last_cone = colored_cones[colored_cones.size() - 2];
  const TwoDVector last_vector = {last_cone.position.x - second_last_cone.position.x,
                                  last_cone.position.y - second_last_cone.position.y};
  for (const auto& cone : uncolored_cones) {
    if (colored_cones.back() == this->colored_blue_cones_.back()) {
      for (auto& colored_cone : this->colored_yellow_cones_) {
        if (cone.position.euclidean_distance(colored_cone.position) < 2.5) {
          continue;
        }
      }
    } else {
      for (auto& colored_cone : this->colored_blue_cones_) {
        if (cone.position.euclidean_distance(colored_cone.position) < 2.5) {
          continue;
        }
      }
    }
    double cost =
        calculate_cost(cone, last_cone, second_last_cone, last_vector,
                       static_cast<double>(n_colored_cones) / static_cast<double>(n_input_cones));
    // TODO: put this value as a parameter
    if (cost < min_cost && cone.position.euclidean_distance(last_cone.position) < 7 &&
        cost < this->config_.max_cost_) {
      min_cost = cost;
      cheapest_cone = cone;
    }
  }
  if (min_cost < std::numeric_limits<double>::max()) {
    uncolored_cones.erase(cheapest_cone);
    colored_cones.push_back(cheapest_cone);
    n_colored_cones++;
    return true;
  } else {
    return false;
  }
}


pair<double, Cone> ConeColoring::best_coloring_cost(std::unordered_set<Cone, std::hash<Cone>>& uncolored_cones, 
std::vector<Cone>& colored_cones, vector<Cone>& oposite_color_cones, int& n_colored_cones, const int n_input_cones) {
  double min_cost = std::numeric_limits<double>::max();
  Cone cheapest_cone;
  const Cone last_cone = colored_cones.back();
  const Cone second_last_cone = colored_cones[colored_cones.size() - 2];
  const TwoDVector last_vector = {last_cone.position.x - second_last_cone.position.x,
                                  last_cone.position.y - second_last_cone.position.y};
  for (const auto& cone : uncolored_cones) {
    
    double mindistance = std::numeric_limits<double>::max();  
    double distance;
      for (auto& colored_cone : oposite_color_cones) {
        distance = cone.position.euclidean_distance(colored_cone.position);
        if (distance < mindistance) {
          mindistance = distance;
        }
        if (distance < 2.0) {
          continue;
        }
      }
    
    double cost =
        ConeColoring::calculate_cost(cone, last_cone, second_last_cone, last_vector,
                       static_cast<double>(n_colored_cones) / static_cast<double>(n_input_cones)) + mindistance*1;
    // TODO: put this value as a parameter
    if (cost < min_cost && cone.position.euclidean_distance(last_cone.position) < 10 &&
        cost < this->config_.max_cost_) {
      min_cost = cost;
      cheapest_cone = cone;
    }
  }
  
  return {min_cost, cheapest_cone};
}



std::vector<std::pair<double, Cone>> ConeColoring::top_coloring_costs(std::unordered_set<Cone, std::hash<Cone>>& uncolored_cones, 
std::vector<Cone>& colored_cones, vector<Cone>& oposite_color_cones, int& n_colored_cones, const int n_input_cones) {

  const Cone last_cone = colored_cones.back();
  const Cone second_last_cone = colored_cones[colored_cones.size() - 2];
  const TwoDVector last_vector = {last_cone.position.x - second_last_cone.position.x,
                                  last_cone.position.y - second_last_cone.position.y};


  std::vector<std::pair<double, Cone>> top_cones;
               
  for (const auto& cone : uncolored_cones) {
    
    double mindistance = std::numeric_limits<double>::max();  
    double distance;
      for (auto& colored_cone : oposite_color_cones) {
        distance = cone.position.euclidean_distance(colored_cone.position);
        if (distance < mindistance) {
          mindistance = distance;
        }
        if (distance < 2.0) {
          continue;
        }
      }
    
    double cost = ConeColoring::calculate_cost(cone, last_cone, second_last_cone, last_vector,
                       static_cast<double>(n_colored_cones) / static_cast<double>(n_input_cones)) + mindistance*1;
    // TODO: put this value as a parameter
    if (cone.position.euclidean_distance(last_cone.position) < 10 &&
        cost < this->config_.max_cost_) {
      top_cones.push_back({cost, cone});
    }
  }
  
  return top_cones;
}





pair<vector<Cone>, vector<Cone>> ConeColoring::dijkstra_search(ColoringCombination initial_state) {
                                                                            
  double outlier_cost_per_cone = 60;    // the cost of accepting to not color a cone
  double max_cost = this->config_.max_cost_;               // the maximum cost of coloring a cone  
  int max_tree_growth = 3; // the maximum number of new cones to consider per level

  std::priority_queue<ColoringCombination, std::vector<ColoringCombination>, ColoringCombinationComparator> pq;
  
  //Continue code
  
  // Push to the stack the first Combination
  pq.push(initial_state);

  // Start loop
  int iterations = 0;
  ColoringCombination current;
  while (!pq.empty())
  {
    iterations++;
    current = pq.top();
    pq.pop();
    if (current.last || current.depth == 15)
    {
      break;
    }

    // Push the possibility of not coloring any cone (outlier cost * n_cones) to the pq in a ConePair with nullpointer
    pq.push({current.blue_cones, current.yellow_cones, current.remaining_cones, current.cost + outlier_cost_per_cone*current.remaining_cones.size(), true, current.depth+1});

    int total_cones = current.blue_cones.size() + current.yellow_cones.size();
    // Calculate all the possible cones to go to from the ConePairs, that are less than the max cost
    vector<pair<double,Cone>> yellow_cones = top_coloring_costs(current.remaining_cones, current.yellow_cones, current.blue_cones, total_cones, total_cones+current.remaining_cones.size());
    vector<pair<double,Cone>> blue_cones = top_coloring_costs(current.remaining_cones, current.blue_cones, current.yellow_cones, total_cones, total_cones+current.remaining_cones.size());

    priority_queue<pair<bool, pair<double,Cone>>, vector<pair<bool, pair<double,Cone>>>, ColoringComparator> sorted_cones;
    for (int i = 0; i < blue_cones.size(); i++)
    {
      sorted_cones.push({true, blue_cones[i]});
    }

    for (int i = 0; i < yellow_cones.size(); i++)
    {
      sorted_cones.push({false, yellow_cones[i]});
    }
    
    // Push the new cone pairs to the pq, only push a maximum of max_tree_growth new cone pairs, the ones with the lowest cost  
    for (int i = 0; i < std::min(max_tree_growth, static_cast<int>(sorted_cones.size())); i++)
    {
      pair<bool, pair<double,Cone>> temporary = sorted_cones.top();
      sorted_cones.pop();
      if (temporary.first){
        vector<Cone> blue_cones = current.blue_cones;
        blue_cones.push_back(temporary.second.second);
        std::unordered_set<Cone, std::hash<Cone>> remaining_cones = current.remaining_cones;
        remaining_cones.erase(temporary.second.second);
        pq.push({blue_cones, current.yellow_cones, remaining_cones, current.cost + temporary.second.first, false, current.depth+1});
      }
      else{
        vector<Cone> yellow_cones = current.yellow_cones;
        yellow_cones.push_back(temporary.second.second);
        std::unordered_set<Cone, std::hash<Cone>> remaining_cones = current.remaining_cones;
        remaining_cones.erase(temporary.second.second);
        pq.push({current.blue_cones, yellow_cones, remaining_cones, current.cost + temporary.second.first, false, current.depth+1});
      }
    }

    if(sorted_cones.size() == 0){
      pq.push({current.blue_cones, current.yellow_cones, current.remaining_cones, current.cost + outlier_cost_per_cone*current.remaining_cones.size(), true, current.depth+1});
    }
    // Pop the first element of the pq and repeat the process


    // Filtrar as melhores opções 
  }
  
  this->colored_blue_cones_ = current.blue_cones;
  this->colored_yellow_cones_ = current.yellow_cones; 
  

  return {this->colored_blue_cones_, this->colored_yellow_cones_};
  }



std::pair<std::vector<Cone>, std::vector<Cone>> ConeColoring::color_cones(std::vector<Cone> cones,
                                                                          const Pose& car_pose) {
  if (!this->config_.use_memory_) {
    this->colored_blue_cones_.clear();
    this->colored_yellow_cones_.clear();
  }
  cones = filter_previously_colored_cones(cones);
  remove_duplicates(cones);

  int n_colored_cones = 0;
  const auto n_input_cones = static_cast<int>(cones.size());
  RCLCPP_DEBUG(rclcpp::get_logger("Planning : ConeColoring"), "Number of received cones: %d",
               n_input_cones);
  std::unordered_set<Cone, std::hash<Cone>> uncolored_cones(cones.begin(), cones.end());

  if (this->colored_blue_cones_.empty() || this->colored_yellow_cones_.empty()) {
    if (cones.size() < 4) {
      RCLCPP_WARN(rclcpp::get_logger("Planning: Cone Coloring"), "No cones found yet.");
      return {{}, {}};
    } else {
      place_initial_cones(uncolored_cones, car_pose, n_colored_cones);
    }
  }

  // Place the new algorithm here, and remove the old one

  // Dijkstra search
  ColoringCombination initial_state;
  initial_state.blue_cones = this->colored_blue_cones_;
  initial_state.yellow_cones = this->colored_yellow_cones_;
  initial_state.remaining_cones = uncolored_cones;
  initial_state.cost = 0;
  initial_state.last = false;

  dijkstra_search(initial_state);




  // bool colouring_blue_cones = true, colouring_yellow_cones = true;
  // // Color blue cones
  // pair<double, Cone> min_blue_cost, min_yellow_cost;
  // while (colouring_blue_cones || colouring_yellow_cones) {
  //   // keep coloring yellow cones while the function "try_to_color_next_cone" returns true (i.e. a
  //   // suitble cone is found)
  //   // colouring_blue_cones = try_to_color_next_cone(uncolored_cones, this->colored_blue_cones_,
  //   //                                               n_colored_cones, n_input_cones);
  //   // colouring_yellow_cones = try_to_color_next_cone(uncolored_cones, this->colored_yellow_cones_,
  //   //                                                 n_colored_cones, n_input_cones);
  //   min_blue_cost = best_coloring_cost(uncolored_cones, this->colored_blue_cones_, this->colored_yellow_cones_,
  //                                      n_colored_cones, n_input_cones);
    
  //   min_yellow_cost = best_coloring_cost(uncolored_cones, this->colored_yellow_cones_, this->colored_blue_cones_,
  //                                      n_colored_cones, n_input_cones);

  //   if (min_blue_cost.first < min_yellow_cost.first) {
  //     if(min_blue_cost.first > this->config_.max_cost_){
  //       break;
  //     }
  //     this->colored_blue_cones_.push_back(min_blue_cost.second);
  //     uncolored_cones.erase(min_blue_cost.second);
  //     n_colored_cones++;
  //   } else {
  //     if(min_yellow_cost.first > this->config_.max_cost_){
  //       break;
  //     }
  //     this->colored_yellow_cones_.push_back(min_yellow_cost.second);
  //     uncolored_cones.erase(min_yellow_cost.second);
  //     n_colored_cones++;
  //   }
  // }

  remove_too_close_cones();

  //// Color yellow cones
  // while (try_to_color_next_cone(uncolored_cones, colored_yellow_cones, n_colored_cones,
  //                               n_input_cones)) {
  //   // keep coloring yellow cones while the function "try_to_color_next_cone" returns true
  //   (i.e.
  //   a
  //   // suitble cone is found)
  // }
  // colored_blue_cones.erase(colored_blue_cones.begin());
  // colored_yellow_cones.erase(colored_yellow_cones.begin());

  if (colored_blue_cones_.size() < 5) {
    RCLCPP_DEBUG(rclcpp::get_logger("Planning : ConeColoring"), "Not enough blue cones found: %ld",
                 colored_blue_cones_.size());
  }
  if (colored_yellow_cones_.size() < 5) {
    RCLCPP_DEBUG(rclcpp::get_logger("Planning : ConeColoring"),
                 "Not enough yellow cones found: %ld", colored_yellow_cones_.size());
  }
  return {this->colored_blue_cones_, this->colored_yellow_cones_};
}
