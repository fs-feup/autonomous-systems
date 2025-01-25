#include <utils/weights.hpp>

double Weights::getSum() const {
  return height_out + height_in + cylinder_radius + cylinder_height + cylinder_npoints + npoints +
         displacement_x + displacement_y + displacement_z + deviation_xoy + deviation_z;
}

void Weights::normalize() {
  double sum = getSum();
  height_out /= sum;
  height_in /= sum;
  cylinder_radius /= sum;
  cylinder_height /= sum;
  cylinder_npoints /= sum;
  npoints /= sum;
  displacement_x /= sum;
  displacement_y /= sum;
  displacement_z /= sum;
  deviation_xoy /= sum;
  deviation_z /= sum;
}