//#include <utils/evaluator_parameters.hpp>
//
// double EvaluatorParameters::getSum() const {
//  return height_out_weight + height_in_weight + cylinder_radius_weight + cylinder_height_weight +
//         cylinder_npoints_weight + npoints_weight + displacement_x_weight + displacement_y_weight
//         + displacement_z_weight + deviation_xoy_weight + deviation_z_weight;
//}
//
// void EvaluatorParameters::normalize_weights() {
//  double sum = getSum();
//  height_out_weight /= sum;
//  height_in_weight /= sum;
//  cylinder_radius_weight /= sum;
//  cylinder_height_weight /= sum;
//  cylinder_npoints_weight /= sum;
//  npoints_weight /= sum;
//  displacement_x_weight /= sum;
//  displacement_y_weight /= sum;
//  displacement_z_weight /= sum;
//  deviation_xoy_weight /= sum;
//  deviation_z_weight /= sum;
//}