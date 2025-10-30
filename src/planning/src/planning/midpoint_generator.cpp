#include "planning/midpoint_generator.hpp"
#include "utils/cone.hpp"

std::vector<std::shared_ptr<Midpoint>>& MidpointGenerator::generate_midpoints(
                    const std::vector<Cone>& cone_array, bool rebuild_all_midpoints) {
     
  triangulations_.clear();
  DT dt;

  midpoints_.clear();

  std::vector<std::shared_ptr<Cone>> filtered_cones;
  filtered_cones.reserve(cone_array.size());
  
  filter_cones(cone_array, filtered_cones, rebuild_all_midpoints);

  // Insert all cone positions into the Delaunay triangulation
  for (const auto& cone : filtered_cones) {
    (void)dt.insert(Point(cone->position.x, cone->position.y));
  }

  // Avoid duplicate midpoints for the same cone pair
  std::map<std::pair<int, int>, std::shared_ptr<Midpoint>> segment_to_midpoint;

  for (auto triangle_it = dt.finite_faces_begin(); triangle_it != dt.finite_faces_end(); ++triangle_it) {
    std::array<std::shared_ptr<Midpoint>, 3> triangle_midpoints;

    // Iterate over the 3 edges of the triangle
    for (int i = 0; i < 3; ++i) {
      Vertex_handle va = triangle_it->vertex((i + 1) % 3);
      Vertex_handle vb = triangle_it->vertex((i + 2) % 3);

      triangle_midpoints[i] = process_triangle_edge(va, vb, filtered_cones, segment_to_midpoint);
    }
    
    // Connect midpoints if they share the same triangle
    connect_triangle_midpoints(triangle_midpoints);
  }

  return midpoints_;
}

void MidpointGenerator::filter_cones(const std::vector<Cone>& cone_array,
                                          std::vector<std::shared_ptr<Cone>>& filtered_cones,
                                          bool rebuild_all_midpoints) {

  if(!rebuild_all_midpoints) {
    for (const auto& cone : cone_array) {
      double dx = cone.position.x - vehicle_pose_.position.x;
      double dy = cone.position.y - vehicle_pose_.position.y;

      double sq_window_radius = config_.sliding_window_radius_ * config_.sliding_window_radius_; 

      if (dx * dx + dy * dy <= sq_window_radius) {
        filtered_cones.push_back(std::make_shared<Cone>(cone));
      }
    }
  }else{
    for (const auto& cone : cone_array) {
      filtered_cones.push_back(std::make_shared<Cone>(cone));
    }
  }

  if (filtered_cones.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("planning"),"[Planning] Not enough cones to compute midpoints");
  }

}

std::shared_ptr<Midpoint> MidpointGenerator::process_triangle_edge(
    const Vertex_handle& va,
    const Vertex_handle& vb,
    std::vector<std::shared_ptr<Cone>>& filtered_cones,
    std::map<std::pair<int, int>, std::shared_ptr<Midpoint>>& segment_to_midpoint) {
  
  Point p1 = va->point();
  Point p2 = vb->point();

  // Find corresponding cone IDs
  int id1 = ::find_cone(filtered_cones, p1.x(), p1.y());
  int id2 = ::find_cone(filtered_cones, p2.x(), p2.y());

  if (id1 == -1 || id2 == -1) {
    return nullptr;
  }

  // Check distance constraints
  if (double squared_distance = CGAL::squared_distance(p1, p2);
    (squared_distance <= config_.minimum_cone_distance_ * config_.minimum_cone_distance_) ||
    (squared_distance >= config_.maximum_cone_distance_ * config_.maximum_cone_distance_)) {
    return nullptr;
  }

  // Use ordered cone IDs to uniquely identify the segment
  auto key = std::minmax(id1, id2);

  // Check if this segment was already processed. If so, return existing midpoint
  if (auto it = segment_to_midpoint.find(key); it != segment_to_midpoint.end()) {
    return it->second;
  }


  // Create new midpoint
  auto midpoint = std::make_shared<Midpoint>(
      CGAL::midpoint(p1, p2), 
      filtered_cones[id1], 
      filtered_cones[id2]
  );
  
  segment_to_midpoint[key] = midpoint;
  midpoints_.push_back(midpoint);
  triangulations_.push_back({p1, p2});
  
  return midpoint;
}

void MidpointGenerator::connect_triangle_midpoints(
    const std::array<std::shared_ptr<Midpoint>, 3>& triangle_midpoints) {
  
  for (int i = 0; i < 3; ++i) {
    if (!triangle_midpoints[i]) {
      continue;
    }
    for (int j = 0; j < 3; ++j) {
      if (i == j || !triangle_midpoints[j]) {
        continue;
      }
      triangle_midpoints[i]->close_points.push_back(triangle_midpoints[j]);
    }
  }
}

const std::vector<std::pair<Point, Point>>& MidpointGenerator::get_triangulations() const {
    return triangulations_;
}


void MidpointGenerator::set_vehicle_pose(const Pose& vehicle_pose){
    vehicle_pose_ = vehicle_pose;
}