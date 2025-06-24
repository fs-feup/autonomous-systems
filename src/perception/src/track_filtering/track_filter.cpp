#include "track_filtering/track_filter.hpp"

TrackFilter::TrackFilter(double min_distance, double max_distance, int n_cones)
    : min_distance_(min_distance), max_distance_(max_distance), n_cones_(n_cones) {}

void TrackFilter::filter(std::vector<PreCone>* cones) {
  auto is_invalid = [this, &cones](const PreCone& cone) {
    int count = 0;
    for (const auto& other_cone : *cones) {
      if (cone == other_cone) continue;
      double distance = cone.get_position().euclidean_distance(other_cone.get_position());
      if (distance < min_distance_) {
        return true;
      }
      if (distance < max_distance_) {
        count++;
      }
    }
    return count < n_cones_;
  };
  cones->erase(std::remove_if(cones->begin(), cones->end(), is_invalid), cones->end());
}
