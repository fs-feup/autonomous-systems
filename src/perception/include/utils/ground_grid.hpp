#pragma once
#include <vector>

class GroundGrid {
public:
  GroundGrid() = default;

  GroundGrid(const double range, const double angle, const double radius,
             const double start_augmentation, const double radius_augmentation, const double fov);

  float get_ground_height(const float x, const float y) const;
  void set_ground_height(const float x, const float y, const float height);
  void reset_grid();

private:
  double range_;
  double angle_;
  double radius_;
  double start_augmentation_;
  double radius_augmentation_;
  double fov_;
  int base_bins_;
  double const_end_;
  int num_slices_;
  int num_bins_;
  std::vector<std::vector<float>> grid_;
};
