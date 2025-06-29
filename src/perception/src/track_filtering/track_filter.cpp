#include "track_filtering/track_filter.hpp"

TrackFilter::TrackFilter(double min_distance, double max_distance, int n_cones)
    : min_distance_(min_distance), max_distance_(max_distance), n_cones_(n_cones) {}

void TrackFilter::filter(std::vector<Cluster>& cones) {
  auto is_invalid = [this, &cones](Cluster& cone) {
    int count = 0;
    for (auto& other_cone : cones) {
      double distance = (cone.get_centroid() - other_cone.get_centroid()).norm();
      if (distance == 0){
        continue;
      }
      if (distance < min_distance_) {
        return true;
      }
      if (distance < max_distance_) {
        count++;
      }
    }
    return count < n_cones_;
  };
  cones.erase(std::remove_if(cones.begin(), cones.end(), is_invalid), cones.end());
}
