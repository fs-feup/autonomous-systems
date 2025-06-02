#include "low_pass_filter/lpf.hpp"

LowPassFilter::LowPassFilter(double alpha, double initial_value)
    : alpha_(alpha), prev_value_(initial_value) {}

double LowPassFilter::filter(double input) {
    prev_value_ = alpha_ * input + (1.0 - alpha_) * prev_value_;
    return prev_value_;
}

void LowPassFilter::reset(double value) {
    prev_value_ = value;
}