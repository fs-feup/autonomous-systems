#include "planning/path_calculation.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

#include "utils/cone.hpp"
using namespace std;

std::pair<Point, Point> PathCalculation::ordered_segment(const Point& a, const Point& b) {
    std::pair<Point, Point> result;
    if (a.x() < b.x() || (a.x() == b.x() && a.y() < b.y())) {
        result = {a, b};
    } else {
        result = {b, a};
    }
    return result;
}


std::pair<double, PathCalculation::MidPoint*> PathCalculation::dfs_cost(int depth,
                                                                        const MidPoint* previous,
                                                                        MidPoint* current,
                                                                        double maxcost) {
  if (depth == 0) {
    return {0, current};  // Return current point if depth is 0
  }

  double min_cost = this->config_.max_cost_ * this->config_.search_depth_;
  MidPoint* min_point = current;  // Default to current point

  for (MidPoint* next : current->close_points) {
    // Avoid revisiting the previous point
    if (next == previous) {
      continue;
    }

    // Distance calculation
    double distance = std::sqrt(std::pow(current->point.x() - next->point.x(), 2) +
                           std::pow(current->point.y() - next->point.y(), 2));

    // Angle calculation
    double angle_with_previous =
        std::atan2(current->point.y() - previous->point.y(), current->point.x() - previous->point.x());
    double angle_with_next =
        std::atan2(next->point.y() - current->point.y(), next->point.x() - current->point.x());

    // Normalize angle to be between 0 and π
    double angle = std::abs(angle_with_next - angle_with_previous);
    if (angle > M_PI) {angle = 2 * M_PI - angle;}

    // Local cost calculation
    double local_cost =
        std::pow(angle, this->config_.angle_exponent_) * this->config_.angle_gain_ +
        std::pow(distance, this->config_.distance_exponent_) * this->config_.distance_gain_;

    // Skip if local cost exceeds maximum allowed cost
    if (local_cost > maxcost) {
      continue;
    }

    // Recursive cost calculation
    auto [cost, selected_point] = dfs_cost(depth - 1, current, next, maxcost);

    // Total cost calculation
    double total_cost = local_cost + cost;

    // Update minimum cost and corresponding point
    if (total_cost < min_cost) {
      min_cost = total_cost;
      min_point = next;
    }
  }

  return {min_cost, min_point};
}

std::vector<PathPoint> PathCalculation::no_coloring_planning(std::vector<Cone>& cone_array, common_lib::structures::Pose pose) {
    std::vector<PathPoint> result;
    std::vector<Point> path;

    if (cone_array.size() < 4) {
        RCLCPP_ERROR(rclcpp::get_logger("planning"), "Not enough cones to create a path.");
        result = {};
    }
    else{
        std::vector<std::unique_ptr<MidPoint>> midPoints;
        std::unordered_map<MidPoint*, std::vector<Point>> triangle_points;
        std::unordered_set<Cone*> discarded_cones;

        createMidPoints(cone_array, midPoints, triangle_points);
        connectMidPoints(midPoints, triangle_points);

        std::unordered_map<Point, MidPoint*, PointHash> point_to_midpoint;
        for (const auto& mp : midPoints) {
            point_to_midpoint[mp->point] = mp.get();
        }

        Point car_point(pose.position.x, pose.position.y);

    

        path_update_counter_++;
        if (path_update_counter_ >= config_.reset_global_path_) {
            global_path_.clear();
            path_update_counter_ = 0;
            RCLCPP_INFO(rclcpp::get_logger("planning"), "Global path reset");
        }
        // Find the closest point in the path to the car
        int cutoff_index = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < global_path_.size(); ++i) {
            double dist = CGAL::squared_distance(global_path_[i], car_point);
            if (dist < min_dist) {
                min_dist = dist;
                cutoff_index = static_cast<int>(i);
            }
        }

        if (cutoff_index == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("planning"), "No valid path points found.");
        }

        path_to_car.clear();
        if (cutoff_index != -1 && cutoff_index > config_.lookback_points_) {
            (void)path_to_car.insert(path_to_car.end(), global_path_.begin(), global_path_.begin() + cutoff_index - config_.lookback_points_);
        }


        std::unordered_set<MidPoint*> visited_midpoints;
        selectInitialPath(path, midPoints, pose, point_to_midpoint, visited_midpoints);
        extendPath(path, midPoints, point_to_midpoint, visited_midpoints, discarded_cones);

        std::vector<PathPoint> path_points;
        for (const auto& point : path) {
            (void)path_points.emplace_back(point.x(), point.y());
        }
        global_path_ = path;  // Update global path with the new path
        result = path_points;
    }
    return result;
}


void PathCalculation::createMidPoints(
    std::vector<Cone>& cone_array,
    std::vector<std::unique_ptr<MidPoint>>& midPoints,
    std::unordered_map<MidPoint*, std::vector<Point>>& triangle_points
) {
    DT dt;
    for (const auto& cone : cone_array) {
        (void)dt.insert(Point(cone.position.x, cone.position.y));
    }

    for (Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
        Vertex_handle v1 = it->first->vertex((it->second + 1) % 3);
        Vertex_handle v2 = it->first->vertex((it->second + 2) % 3);
        Point p1 = v1->point();
        Point p2 = v2->point();

        int id1 = ::find_cone(cone_array, p1.x(), p1.y());
        int id2 = ::find_cone(cone_array, p2.x(), p2.y());

        if (id1 == -1 || id2 == -1){
            continue;
        }

        double sq_dist = CGAL::squared_distance(p1, p2);
        
        if (double min_dist = config_.minimum_cone_distance_; sq_dist <= min_dist * min_dist) {
            continue;
        } 
        if (double max_dist = config_.maximum_cone_distance_; sq_dist >= max_dist * max_dist) {
            continue;
        }

        auto midPoint = std::make_unique<MidPoint>(
            MidPoint{Point((p1.x() + p2.x()) / 2, (p1.y() + p2.y()) / 2), {}, &cone_array[id1], &cone_array[id2]}
        );
        MidPoint* mid_ptr = midPoint.get();
        midPoints.push_back(std::move(midPoint));

        std::vector<std::pair<Point, Point>> neighbor_edges;
        auto collect_edges = [&](auto face, int skip_edge_index) {
            for (int i = 0; i < 3; ++i) {
                if (i == skip_edge_index){
                    continue;
                }
                (void)neighbor_edges.emplace_back(face->vertex((i + 1) % 3)->point(), face->vertex((i + 2) % 3)->point());
            }
        };
        collect_edges(it->first, it->second);
        
        if (auto neighbor_face = it->first->neighbor(it->second); !dt.is_infinite(neighbor_face)) {
            collect_edges(neighbor_face, neighbor_face->index(it->first));
        }

        triangle_points[mid_ptr] = {};
        for (const auto& [pa, pb] : neighbor_edges) {
            triangle_points[mid_ptr].push_back(pa);
            triangle_points[mid_ptr].push_back(pb);
        }
    }
}

void PathCalculation::connectMidPoints(
    const std::vector<std::unique_ptr<MidPoint>>& midPoints,
    const std::unordered_map<MidPoint*, std::vector<Point>>& triangle_points
) {
    std::unordered_map<std::pair<Point, Point>, MidPoint*, PairHash> segment_to_midpoint;
    for (const auto& q : midPoints) {
        segment_to_midpoint[ordered_segment(
            Point(q->cone1->position.x, q->cone1->position.y),
            Point(q->cone2->position.x, q->cone2->position.y))] = q.get();
    }

    for (const auto& p : midPoints) {
        auto& points = triangle_points.at(p.get());
        std::unordered_set<std::pair<Point, Point>, PairHash> seen_segments;

        size_t i = 0;
        while (i + 1 < points.size()) {
            auto seg = ordered_segment(points[i], points[i + 1]);
            i += 2;
            if (!seen_segments.insert(seg).second) {
                continue;
            }

            auto it = segment_to_midpoint.find(seg);
            if (it != segment_to_midpoint.end() && it->second != p.get()) {
                p->close_points.push_back(it->second);
            }
        }
    }
}

void PathCalculation::selectInitialPath(
    std::vector<Point>& path,
    const std::vector<std::unique_ptr<MidPoint>>& midPoints,
    const common_lib::structures::Pose& pose,
    const std::unordered_map<Point, MidPoint*, PointHash>& point_to_midpoint,
    std::unordered_set<MidPoint*>& visited_midpoints
) {

    if (path_to_car.size() > 2) {

        RCLCPP_INFO(rclcpp::get_logger("planning"), "Selecting initial path from %zu points.", path_to_car.size());

        Point snapped_first = path_to_car[0];
        
        if (MidPoint* mp_first = find_nearest_point(snapped_first, point_to_midpoint, config_.tolerance_); mp_first) {
            snapped_first = mp_first->point;
            (void)visited_midpoints.insert(mp_first);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("planning"), "No valid midpoints found for the first point.");
        }
        path.push_back(snapped_first);
        Point last_point = snapped_first;

        for (std::size_t i = 1; i < path_to_car.size(); ++i) {
            Point current_point = path_to_car[i];
            MidPoint* current_mp = find_nearest_point(current_point, point_to_midpoint, config_.tolerance_);
            if (current_mp){
                current_point = current_mp->point;
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("planning"), "No valid midpoints found for the current point.");
            }    

            double distance = CGAL::sqrt(CGAL::squared_distance(last_point, current_point));
            if (distance > config_.tolerance_ && (current_mp == nullptr || visited_midpoints.count(current_mp) == 0)) {
                path.push_back(current_point);
                

                last_point = current_point;
                if (current_mp) {
                    (void)visited_midpoints.insert(current_mp);
                }
            }
        }
    } else {
        updateAnchorPoint(pose);
        auto [first, second] = findPathStartPoints(midPoints, anchor_point_);
        if (first != nullptr && second != nullptr) {
            path.push_back(first->point);
            path.push_back(second->point);
            (void)visited_midpoints.insert(first);
            (void)visited_midpoints.insert(second);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("planning"), "Failed to find valid starting points.");
        }
    }
}
void PathCalculation::extendPath(
    std::vector<Point>& path,
    const std::vector<std::unique_ptr<MidPoint>>& midPoints,
    const std::unordered_map<Point, MidPoint*, PointHash>& point_to_midpoint,
    std::unordered_set<MidPoint*>& visited_midpoints,
    std::unordered_set<Cone*>& discarded_cones
) {
    int n_points = 0;

    while (true) {
        double worst_cost = config_.max_cost_ * config_.search_depth_;
        const auto& prev = path[path.size() - 2];
        const auto& last = path.back();

        const MidPoint* prev_mp = find_nearest_point(prev, point_to_midpoint, config_.tolerance_);
        MidPoint* last_mp = find_nearest_point(last, point_to_midpoint, config_.tolerance_);

        if (prev_mp == nullptr || last_mp == nullptr) {
            RCLCPP_ERROR(rclcpp::get_logger("planning"), "No valid midpoints found for path extension.");
            break;
        }

        auto [best_cost, best_point] = dfs_cost(config_.search_depth_, prev_mp, last_mp, config_.max_cost_);
        if (best_cost > worst_cost || best_point == nullptr || visited_midpoints.count(best_point) > 0){
            break;
        }

        path.push_back(best_point->point);
        (void)visited_midpoints.insert(best_point);
        n_points++;
        if (n_points > config_.max_points_){
            break;
        }

        if (path.size() > 2) {
            discard_cones_along_path(path, midPoints, point_to_midpoint, discarded_cones);
        }
    }
}

void PathCalculation::discard_cones_along_path(
    const std::vector<Point>& path,
    const std::vector<std::unique_ptr<MidPoint>>& midPoints,
    const std::unordered_map<Point, MidPoint*, PointHash>& point_to_midpoint,
    std::unordered_set<Cone*>& discarded_cones
) {
    const auto& last = path[path.size() - 2];
    const auto& current = path.back();

    MidPoint* last_mp = find_nearest_point(last, point_to_midpoint, config_.tolerance_);
    const MidPoint* current_mp = find_nearest_point(current, point_to_midpoint, config_.tolerance_);

    if (last_mp == nullptr && current_mp == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("planning"), "No valid midpoints found for discarding cones.");
    }


    Cone* discarded_cone = nullptr;

    if (last_mp->cone1 == current_mp->cone1 || last_mp->cone1 == current_mp->cone2) {
        if (last_mp->cone2 != current_mp->cone1 && last_mp->cone2 != current_mp->cone2) {
            discarded_cone = last_mp->cone2;
        }
    } else if (last_mp->cone2 == current_mp->cone1 || last_mp->cone2 == current_mp->cone2) {
        if (last_mp->cone1 != current_mp->cone1 && last_mp->cone1 != current_mp->cone2) {
            discarded_cone = last_mp->cone1;
        }
    } else {
        RCLCPP_DEBUG(rclcpp::get_logger("planning"), "No valid cones found for discarding.");
    }


    (void)discarded_cones.insert(discarded_cone);

    for (auto& mp : midPoints) {
        if (!mp->valid){
            continue;
        }
        if (discarded_cones.count(mp->cone1) > 0 || discarded_cones.count(mp->cone2) > 0) {
            mp->valid = false;
        }
    }

    for (auto& mp : midPoints) {
        if (!mp->valid){
            continue;
        }
        (void)mp->close_points.erase(
            std::remove_if(mp->close_points.begin(), mp->close_points.end(),
                [](const MidPoint* neighbor) { return !neighbor->valid; }),
            mp->close_points.end()
        );
    }
}


void PathCalculation::updateAnchorPoint(const common_lib::structures::Pose& pose) {
  if (!anchor_point_set_) {
    anchor_point_ = pose;
    anchor_point_set_ = true; 
  }
}

std::pair<PathCalculation::MidPoint*, PathCalculation::MidPoint*>
PathCalculation::findPathStartPoints(const std::vector<std::unique_ptr<MidPoint>>& mid_points,
                                     const common_lib::structures::Pose& anchor_pose) {
  std::pair<MidPoint*, MidPoint*> result{nullptr, nullptr};

  // Find the first point closest to the anchor pose
  MidPoint* first = nullptr;
  double min_dist = std::numeric_limits<double>::max();
  for (const auto& p : mid_points) {
    double dx = p->point.x() - anchor_pose.position.x;
    double dy = p->point.y() - anchor_pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      first = p.get();
    }
  }

  if (first != nullptr) {
    // Project a point based on pose orientation
    MidPoint projected = {
      Point(first->point.x() + std::cos(anchor_pose.orientation) * config_.projected_point_distance_,
            first->point.y() + std::sin(anchor_pose.orientation) * config_.projected_point_distance_),
      {}, nullptr, nullptr
    };

    // Find second point based on projected point
    MidPoint* second = findSecondPoint(mid_points, first, projected, anchor_pose);
    result = {first, second};
  }

  return result;
}


PathCalculation::MidPoint* PathCalculation::findSecondPoint(
    const std::vector<std::unique_ptr<MidPoint>>& mid_points, const MidPoint* first,
    const MidPoint& projected, const common_lib::structures::Pose& anchor_pose) {
  MidPoint* second = nullptr;
  double min_dist = std::numeric_limits<double>::max();

  // Calculate the car's direction vector
  double car_direction_x = std::cos(anchor_pose.orientation);
  double car_direction_y = std::sin(anchor_pose.orientation);

  for (auto& p : mid_points) {
    if (p.get() == first) {continue;}

    // Compute vector from the first point to this candidate point
    double dx = p->point.x() - first->point.x();
    double dy = p->point.y() - first->point.y();

    // Dot product checks whether the candidate is in front of the first point
    if ((dx * car_direction_x + dy * car_direction_y) <= 0.0) {continue;}  // Skip points not in front

    double dist = std::sqrt(std::pow(p->point.x() - projected.point.x(), 2) +
                       std::pow(p->point.y() - projected.point.y(), 2));
    if (dist < min_dist) {
      min_dist = dist;
      second = p.get();
    }
  }

  return second;
}

PathCalculation::MidPoint* PathCalculation::find_nearest_point(
    const Point& target,
    const std::unordered_map<Point, MidPoint*, PointHash>& map,
    double tolerance) 
{
    double min_dist_sq = tolerance * tolerance;
    MidPoint* nearest = nullptr;

    for (const auto& [pt, mp] : map) {
        double dx = pt.x() - target.x();
        double dy = pt.y() - target.y();
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq <= min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest = mp;
        }
    }

    return nearest; // nullptr if none within tolerance
}


std::vector<PathPoint> PathCalculation::process_delaunay_triangulations(
    std::pair<std::vector<Cone>, std::vector<Cone>> refined_cones) const {
  // merge left and right cones for next step
  // RCLCPP_WARN(rclcpp::get_logger("planning"), "Refined cones: %d blue, %d yellow",
  //            static_cast<int>(refined_cones.first.size()),
  //            static_cast<int>(refined_cones.second.size()));
  std::vector<Cone> cones;
  cones.reserve(refined_cones.first.size() + refined_cones.second.size());
  cones.insert(cones.end(), refined_cones.first.begin(), refined_cones.first.end());
  cones.insert(cones.end(), refined_cones.second.begin(), refined_cones.second.end());

  std::vector<PathPoint> unordered_path;
  unordered_path.reserve(cones.size());

  // Create a Delaunay triangulation
  DT dt;
  // Insert cones coordinates into the Delaunay triangulation
  for (const Cone& cone : cones) {
    dt.insert(Point(cone.position.x, cone.position.y));
  }

  // Process valid triangulations and add positions to unordered_path
  for (DT::Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
    // Extract vertices' coordinates from both edges
    double x1 = it->first->vertex((it->second + 1) % 3)->point().x();
    double y1 = it->first->vertex((it->second + 1) % 3)->point().y();
    double x2 = it->first->vertex((it->second + 2) % 3)->point().x();
    double y2 = it->first->vertex((it->second + 2) % 3)->point().y();

    // Find corresponding cones for the vertices
    int id_cone1 = find_cone(cones, x1, y1);
    int id_cone2 = find_cone(cones, x2, y2);
    // Check both cones have been found
    if (id_cone1 == -1 || id_cone2 == -1) {
      RCLCPP_INFO(rclcpp::get_logger("planning"), "Cone not found in triangulations");
      continue;
    }

    Cone cone1 = cones[id_cone1];
    Cone cone2 = cones[id_cone2];
    // If cones are from different sides
    if (cone1.color != cone2.color) {
      // Calculate the midpoint between the two cones
      double x_dist = cone2.position.x - cone1.position.x;
      double y_dist = cone2.position.y - cone1.position.y;
      double dist = std::sqrt(std::pow(x_dist, 2) + std::pow(y_dist, 2));

      if (dist < config_.dist_threshold_) {
        PathPoint pt = PathPoint(cone1.position.x + x_dist / 2, cone1.position.y + y_dist / 2);
        unordered_path.push_back(pt);
      }
    }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("planning"), "Unordered path size: %d",
               static_cast<int>(unordered_path.size()));

  return unordered_path;
}

std::vector<PathPoint> PathCalculation::getGlobalPath() const {
    std::vector<PathPoint> path_points;


    for (const auto& pt : path_to_car) {
        (void)path_points.emplace_back(pt.x(), pt.y());
    }    

    return path_points;
}

std::vector<PathPoint> PathCalculation::skidpad_path(const std::vector<Cone>& cone_array,
                                                     common_lib::structures::Pose pose) {
    std::vector<PathPoint> result;

    if (!path_orientation_corrected_) {
        const std::string file = "./src/planning/src/utils/skidpad.txt";
        const std::string conesfile = "./src/planning/src/utils/skidpadcones.txt";

        // 1. Read reference cones from file
        std::ifstream cfile(conesfile);
        std::vector<std::pair<double, double>> reference_cones;
        std::string line;
        while (std::getline(cfile, line)) {
            std::istringstream iss(line);
            double x = 0.0, y = 0.0, z = 0.0;
            if (iss >> x >> y >> z) {
                reference_cones.emplace_back(x, y);
            } else {
                break;
            }
        }

        if (reference_cones.size() < static_cast<std::size_t>(config_.skidpad_minimum_cones_) ||
            cone_array.size() < static_cast<std::size_t>(config_.skidpad_minimum_cones_)) {
            RCLCPP_ERROR(rclcpp::get_logger("planning"), "Not enough cones to perform ICP alignment.");
            result.clear();  // vazio para indicar falha
        } else {
            // 2. Read hardcoded path
            std::ifstream infile(file);
            while (std::getline(infile, line)) {
                std::istringstream iss(line);
                double x = 0.0, y = 0.0, v = 0.0;
                if (iss >> x >> y >> v) {
                    result.emplace_back(x + config_.skidpad_tolerance_, y, v);
                } else {
                    break;
                }
            }

            // 3. Convert cone_array (LiDAR) to source cloud
            pcl::PointCloud<pcl::PointXYZ> cloud_source;
            cloud_source.reserve(cone_array.size());
            for (const auto& cone : cone_array) {
                cloud_source.emplace_back(cone.position.x, cone.position.y, 0.0);
            }

            // 4. Convert reference cones to target cloud
            pcl::PointCloud<pcl::PointXYZ> cloud_target;
            cloud_target.reserve(reference_cones.size());
            for (const auto& [x, y] : reference_cones) {
                cloud_target.emplace_back(x, y, 0.0);
            }

            // 5. Run ICP alignment
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(cloud_source.makeShared());
            icp.setInputTarget(cloud_target.makeShared());
            icp.setMaxCorrespondenceDistance(10);
            icp.setMaximumIterations(std::numeric_limits<int>::max());
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-3);

            pcl::PointCloud<pcl::PointXYZ> Final;
            icp.align(Final);

            if (!icp.hasConverged()) {
                RCLCPP_ERROR(rclcpp::get_logger("planning"), "ICP did not converge.");
                result.clear();  // vazio para indicar falha
            } else {
                Eigen::Matrix4f icp_transform = icp.getFinalTransformation();  // LiDAR → MAP
                Eigen::Matrix4f map_to_lidar_transform = icp_transform.inverse();

                // 6. Apply transformation to hardcoded path
                for (auto& point : result) {
                    Eigen::Vector4f p(point.position.x, point.position.y, 0.0f, 1.0f);
                    Eigen::Vector4f transformed = map_to_lidar_transform * p;
                    point.position.x = transformed[0];
                    point.position.y = transformed[1];
                }

                predefined_path_ = result;
                path_orientation_corrected_ = true;
            }
        }
    }

    // Caso a correção já tenha ocorrido, ou após corrigir, remove pontos próximos
    while (!predefined_path_.empty() &&
           pose.position.euclidean_distance(predefined_path_.front().position) < 1) {
        predefined_path_.erase(predefined_path_.begin());
    }

    // Se a predefined_path_ estiver vazia ou menor que 70, evita acesso inválido
    size_t path_size = predefined_path_.size();
    size_t count = (path_size >= 70) ? 70 : path_size;

    return std::vector<PathPoint>(predefined_path_.begin(), predefined_path_.begin() + count);
}
