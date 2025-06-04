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

std::pair<Point, Point> ordered_segment(const Point& a, const Point& b) {
  if (a.x() < b.x() || (a.x() == b.x() && a.y() < b.y())) {
      return {a, b};
  }
  return {b, a};
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


PathCalculation::MidPoint* PathCalculation::find_matching_midpoint_pcl(const Point& query, 
                                     const pcl::KdTreeFLANN<pcl::PointXYZ>& kd_tree, 
                                     const std::vector<MidPoint*>& index_map,
                                     double radius) 
{
    pcl::PointXYZ searchPoint(query.x(), query.y(), 0.0f);
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;

    if (kd_tree.radiusSearch(searchPoint, radius, k_indices, k_sqr_distances) > 0) {
        return index_map[k_indices[0]]; // Return closest match
    }

    return nullptr;
}

std::vector<PathPoint> PathCalculation::no_coloring_planning(std::vector<Cone>& cone_array, common_lib::structures::Pose pose) {
    std::vector<PathPoint> result;
    std::vector<Point> path;

    if (cone_array.size() >= 4) {
        std::vector<std::unique_ptr<MidPoint>> midPoints;
        std::unordered_set<Cone*> discarded_cones;

        DT dt;
        for (const auto& cone : cone_array) {
            dt.insert(Point(cone.position.x, cone.position.y));
        }

        std::unordered_map<MidPoint*, std::vector<Point>> triangle_points;

        for (Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
            Vertex_handle v1 = it->first->vertex((it->second + 1) % 3);
            Vertex_handle v2 = it->first->vertex((it->second + 2) % 3);
            Point p1 = v1->point();
            Point p2 = v2->point();

            int id1 = find_cone(cone_array, p1.x(), p1.y());
            int id2 = find_cone(cone_array, p2.x(), p2.y());

            if (id1 == -1 || id2 == -1) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Cone not found in triangulation");
                continue;
            }

            double sq_dist = CGAL::squared_distance(p1, p2);
            double min_dist = config_.minimum_cone_distance_;
            if (sq_dist > min_dist * min_dist) {
                auto midPoint = std::make_unique<MidPoint>(
                    MidPoint{Point((p1.x() + p2.x()) / 2, (p1.y() + p2.y()) / 2), {}, &cone_array[id1], &cone_array[id2]}
                );
                MidPoint* mid_ptr = midPoint.get();
                midPoints.push_back(std::move(midPoint));

                std::vector<std::pair<Point, Point>> neighbor_edges;
                auto collect_edges = [&](auto face, int skip_edge_index) {
                    for (int i = 0; i < 3; ++i) {
                        if (i == skip_edge_index) continue;
                        Vertex_handle va = face->vertex((i + 1) % 3);
                        Vertex_handle vb = face->vertex((i + 2) % 3);
                        neighbor_edges.push_back({va->point(), vb->point()});
                    }
                };

                collect_edges(it->first, it->second);
                auto neighbor_face = it->first->neighbor(it->second);
                if (!dt.is_infinite(neighbor_face)) {
                    int neighbor_index = neighbor_face->index(it->first);
                    collect_edges(neighbor_face, neighbor_index);
                }

                triangle_points[mid_ptr] = {};
                for (auto& [pa, pb] : neighbor_edges) {
                    triangle_points[mid_ptr].push_back(pa);
                    triangle_points[mid_ptr].push_back(pb);
                }
            }
        }

        std::unordered_map<std::pair<Point, Point>, MidPoint*, pair_hash> segment_to_midpoint;
        for (auto& q : midPoints) {
            Point p1(q->cone1->position.x, q->cone1->position.y);
            Point p2(q->cone2->position.x, q->cone2->position.y);
            segment_to_midpoint[ordered_segment(p1, p2)] = q.get();
        }

        for (auto& p : midPoints) {
            auto& points = triangle_points[p.get()];
            std::unordered_set<std::pair<Point, Point>, pair_hash> seen_segments;

            for (size_t i = 0; i + 1 < points.size(); i += 2) {
                Point a = points[i];
                Point b = points[i + 1];
                auto seg = ordered_segment(a, b);

                if (!seen_segments.insert(seg).second) continue;

                auto it = segment_to_midpoint.find(seg);
                if (it != segment_to_midpoint.end() && it->second != p.get()) {
                    p->close_points.push_back(it->second);
                }
            }
        }


        pcl::PointCloud<pcl::PointXYZ>::Ptr kd_tree_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<MidPoint*> kd_tree_midpoints;

        for (const auto& mp : midPoints) {
            pcl::PointXYZ pcl_point(mp->point.x(), mp->point.y(), 0.0f);
            kd_tree_cloud->points.push_back(pcl_point);
            kd_tree_midpoints.push_back(mp.get());
        }

        pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
        kd_tree.setInputCloud(kd_tree_cloud);
        // --------------------------------------------------------------

        int lookback = config_.lookback_points_;
        int total = static_cast<int>(global_path_.size());
        int usable_points = total - lookback - 1;

        path.clear();

        RCLCPP_INFO(rclcpp::get_logger("planning"), "Usable points: %d", usable_points);

        std::unordered_set<MidPoint*> visited_midpoints;

    if (usable_points > 5) {
        constexpr double min_distance = 1.0;

        Point snapped_first = global_path_[0];
        MidPoint* mp_first = find_matching_midpoint_pcl(snapped_first, kd_tree, kd_tree_midpoints, 1);
        if (mp_first) {
            snapped_first = mp_first->point;
            visited_midpoints.insert(mp_first);
        }

        path.push_back(snapped_first);
        Point last_point = snapped_first;

        for (int i = 1; i < usable_points; ++i) {
            Point current_point = global_path_[i];

            MidPoint* current_mp = find_matching_midpoint_pcl(current_point, kd_tree, kd_tree_midpoints, 1);
            if (current_mp) {
                current_point = current_mp->point;
            }

            double dx = current_point.x() - last_point.x();
            double dy = current_point.y() - last_point.y();
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance > min_distance) {
                if (!current_mp || visited_midpoints.count(current_mp) == 0) {
                    path.push_back(current_point);
                    last_point = current_point;
                    if (current_mp) visited_midpoints.insert(current_mp);
                }
            }
        }
    } else {
        updateAnchorPoint(pose);
        auto [first, second] = findPathStartPoints(midPoints, anchor_point_);
        if (first && second) {
            path.push_back(first->point);
            path.push_back(second->point);
            visited_midpoints.insert(first);
            visited_midpoints.insert(second);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("planning"), "Failed to find valid starting points.");
            return {};
        }
    }

    int n_points = 0;

    while (true) {
        double worst_cost = this->config_.max_cost_ * this->config_.search_depth_;

        const auto& prev = path[path.size() - 2];
        const auto& last = path.back();

        MidPoint* prev_mp = find_matching_midpoint_pcl(prev, kd_tree, kd_tree_midpoints, 1);
        MidPoint* last_mp = find_matching_midpoint_pcl(last, kd_tree, kd_tree_midpoints, 1);

        auto [best_cost, best_point] = dfs_cost(
            this->config_.search_depth_,
            prev_mp,
            last_mp,
            this->config_.max_cost_);

        if (best_cost > worst_cost || !best_point) {
            break;
        }

        if (visited_midpoints.count(best_point)) {
            break;
        }

        n_points++;
        if (n_points > this->config_.max_points_) {
            break;
        }

        path.push_back(best_point->point);
        visited_midpoints.insert(best_point);

        // Atualização dos cones descartados
        if (path.size() > 2) {
            const auto& last = path[path.size() - 2];
            const auto& current = path.back();

            MidPoint* current_mp = find_matching_midpoint_pcl(current, kd_tree, kd_tree_midpoints, 0.4);
            MidPoint* last_mp = find_matching_midpoint_pcl(last, kd_tree, kd_tree_midpoints, 0.4);

            Cone* discarded_cone = nullptr;
            if (last_mp && current_mp) {
                if (last_mp->cone1 == current_mp->cone1 || last_mp->cone1 == current_mp->cone2) {
                    if (last_mp->cone2 != current_mp->cone1 && last_mp->cone2 != current_mp->cone2) {
                        discarded_cone = last_mp->cone2;
                    }
                } else if (last_mp->cone2 == current_mp->cone1 || last_mp->cone2 == current_mp->cone2) {
                    if (last_mp->cone1 != current_mp->cone1 && last_mp->cone1 != current_mp->cone2) {
                        discarded_cone = last_mp->cone1;
                    }
                }

                if (discarded_cone) {
                    discarded_cones.insert(discarded_cone);
                    for (auto& mp : midPoints) {
                        if (!mp->valid) continue;
                        if (discarded_cones.count(mp->cone1) || discarded_cones.count(mp->cone2)) {
                            mp->valid = false;
                        }
                    }
                    for (auto& mp : midPoints) {
                        if (!mp->valid) continue;
                        mp->close_points.erase(
                            std::remove_if(mp->close_points.begin(), mp->close_points.end(),
                                [](MidPoint* neighbor) { return !neighbor->valid; }),
                            mp->close_points.end()
                        );
                    }
                }
            }
        }
    }
    }

    global_path_.clear();
    global_path_ = path;

    RCLCPP_INFO(rclcpp::get_logger("planning"), "Generated path with %zu points", path.size());

    std::vector<PathPoint> path_points;
    for (const auto& point : path) {
        path_points.emplace_back(point.x(), point.y());
    }

    return path_points;
}


PathCalculation::MidPoint* PathCalculation::find_nearest_point(
    const Point& target,
    const std::unordered_map<Point, MidPoint*, point_hash>& map,
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



// std::vector<std::unique_ptr<PathCalculation::MidPoint>> PathCalculation::createMidPointsFromTriangulation(
//     const std::vector<Cone>& cone_array) {
//   std::vector<std::unique_ptr<MidPoint>> mid_points;
//   DT dt;

//   // Insert cones into triangulation
//   for (auto cone : cone_array) {
//     auto _ = dt.insert(Point(cone.position.x, cone.position.y));
//   }

//   // Create midpoints
//   for (DT::Finite_edges_iterator it = dt.finite_edges_begin(); it != dt.finite_edges_end(); ++it) {
//     Point p1 = it->first->vertex((it->second + 1) % 3)->point();
//     Point p2 = it->first->vertex((it->second + 2) % 3)->point();

//     auto midPoint = std::make_unique<MidPoint>(
//         MidPoint{Point((p1.x() + p2.x()) / 2, (p1.y() + p2.y()) / 2), {}});

//     double dist = std::sqrt(std::pow(p1.x() - p2.x(), 2) + std::pow(p1.y() - p2.y(), 2));
//     if (dist > config_.minimum_cone_distance_) { // constrained by the rules (cones 3m apart)
//       mid_points.push_back(std::move(midPoint));
//     }
//   }

//   return mid_points;
// }


// void PathCalculation::establishMidPointConnections(
//     const std::vector<std::unique_ptr<MidPoint>>& mid_points) {
//   // Comparison function for priority queue
//   auto cmp = [](const std::pair<double, MidPoint*>& distance_to_midpoint1,
//                 const std::pair<double, MidPoint*>& distance_to_midpoint2) {
//     return distance_to_midpoint1.first > distance_to_midpoint2.first;
//   };

//   // Populate close points for each midpoint
//   for (const auto& p : mid_points) {
//     std::priority_queue<std::pair<double, MidPoint*>, std::vector<std::pair<double, MidPoint*>>,
//                         decltype(cmp)>
//         pq(cmp);

//     for (const auto& q : mid_points) {
//       if (p.get() == q.get()) {continue;}

//       double dist = std::sqrt(std::pow(p->point.x() - q->point.x(), 2) +
//                               std::pow(p->point.y() - q->point.y(), 2));
//       pq.push({dist, q.get()});
//     }

//     // Take top 5 close points as pointers
//     for (int i = 0; i < 6; i++) {
//       if (pq.empty()) {break;}
//       p->close_points.push_back(pq.top().second);
//       pq.pop();
//     }
//   }
// }

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



std::vector<PathPoint> PathCalculation::convertToPathPoints(const std::vector<MidPoint*>& path) {
  std::vector<PathPoint> final_path;
  for (const MidPoint* p : path) {
    final_path.push_back(PathPoint(p->point.x(), p->point.y()));
  }

  return final_path;
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
  for (const auto& pt : global_path_) {
      path_points.emplace_back(pt.x(), pt.y());
  }
  return path_points;
}

std::vector<PathPoint> PathCalculation::skidpad_path(std::vector<Cone>& cone_array,
                                                     common_lib::structures::Pose pose) {
    std::string file = "./src/planning/src/utils/skidpad.txt";
    std::ifstream infile(file);
    std::string line;
    std::vector<PathPoint> hardcoded_path_;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        double x, y, v;
        if (!(iss >> x >> y >> v)) break;
        hardcoded_path_.emplace_back(x, y, v);
    }

    if (cone_array.size() < 4 && !has_icp_transform_) {
        throw std::runtime_error("Insufficient cones and no previous ICP transform");
    }

    // Prever midpoints
    std::vector<std::pair<double, double>> midpoint_coords;
    std::vector<bool> used(cone_array.size(), false);

    for (size_t i = 0; i < cone_array.size(); ++i) {
        if (used[i]) continue;
        const auto& cone_i = cone_array[i];
        double best_dist = std::numeric_limits<double>::max();
        int best_j = -1;

        for (size_t j = i + 1; j < cone_array.size(); ++j) {
            if (used[j]) continue;
            const auto& cone_j = cone_array[j];

            double dist_i = std::hypot(cone_i.position.x - pose.position.x,
                                       cone_i.position.y - pose.position.y);
            double dist_j = std::hypot(cone_j.position.x - pose.position.x,
                                       cone_j.position.y - pose.position.y);
            double delta_dist = std::abs(dist_i - dist_j);
            if (delta_dist > 1.5) continue;

            double dx = cone_i.position.x - cone_j.position.x;
            double dy = cone_i.position.y - cone_j.position.y;
            double cone_dist = std::hypot(dx, dy);
            if (cone_dist < 0.5 || cone_dist > 5.0) continue;

            if (cone_dist < best_dist) {
                best_dist = cone_dist;
                best_j = j;
            }
        }

        if (best_j != -1) {
            used[i] = true;
            used[best_j] = true;
            double mx = (cone_array[i].position.x + cone_array[best_j].position.x) / 2.0;
            double my = (cone_array[i].position.y + cone_array[best_j].position.y) / 2.0;
            midpoint_coords.emplace_back(mx, my);
        }
    }

    // Criar local_path com base nos midpoints
    std::vector<PathPoint> local_path;
    for (const auto& pt : hardcoded_path_) {
        for (const auto& [x, y] : midpoint_coords) {
            if (std::hypot(pt.position.x - x, pt.position.y - y) < 5.0) {
                local_path.push_back(pt);
                break;
            }
        }
    }

    if (local_path.size() >= 5 && midpoint_coords.size() >= 2) {
        // Converter para nuvens PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& pt : local_path) {
            cloud_source->points.emplace_back(pt.position.x, pt.position.y, 0.0);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& [x, y] : midpoint_coords) {
            cloud_target->points.emplace_back(x, y, 0.0);
        }

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_source);
        icp.setInputTarget(cloud_target);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);

        if (icp.hasConverged()) {
            last_icp_transform_ = icp.getFinalTransformation();
            has_icp_transform_ = true;
           
        }
    }

    if (!has_icp_transform_) {
        throw std::runtime_error("No ICP transform available");
    }

    // Só atualizar predefined_path_ se ainda não foi alinhado
    if (!path_aligned_ && has_icp_transform_) {
      predefined_path_.clear();
      for (auto pt : hardcoded_path_) {
          Eigen::Vector4f p(pt.position.x, pt.position.y, 0.0f, 1.0f);
          Eigen::Vector4f transformed = last_icp_transform_ * p;
          pt.position.x = transformed[0];
          pt.position.y = transformed[1];
          predefined_path_.push_back(pt);
      }
      path_aligned_ = true;
    }


    
    while (predefined_path_.size() > 1 &&
           pose.position.euclidean_distance(predefined_path_[0].position) >
           pose.position.euclidean_distance(predefined_path_[1].position)) {
        predefined_path_.erase(predefined_path_.begin());
    }

    size_t N = std::min<size_t>(70, predefined_path_.size());
    return std::vector<PathPoint>(predefined_path_.begin(), predefined_path_.begin() + N);
}
