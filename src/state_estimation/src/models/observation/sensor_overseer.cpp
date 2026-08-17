#include "models/observation/sensor_overseer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

SensorOverseer::SensorOverseer(std::vector<SensorSpec> specs)
    : specs_(std::move(specs)), states_(specs_.size()) {}

void SensorOverseer::update(std::size_t id, const std::vector<double>& values,
                            const rclcpp::Time& stamp) {
  ChannelState& st = states_[id];
  const SensorSpec& spec = specs_[id];
  // Header stamps are only ever compared against this sensor's own previous stamp, so a
  // sensor whose clock differs from the others (or from the node clock) stays consistent.
  const int64_t stamp_ns = stamp.nanoseconds();

  bool out_of_range = false;
  bool rate_violation = false;
  const std::size_t n = std::min(values.size(), spec.signals.size());
  for (std::size_t i = 0; i < n; ++i) {
    const SignalSpec& sig = spec.signals[i];

    if (values[i] < sig.range_min || values[i] > sig.range_max) {
      out_of_range = true;
    }

    // Rate-of-change check is skipped on the first sample, where no prior value exists.
    if (st.has_prev && i < st.prev.size()) {
      const double dt = (stamp_ns - st.last_stamp_ns) * 1e-9;
      if (dt > 1e-9 && std::abs(values[i] - st.prev[i]) / dt > sig.max_rate_of_change) {
        rate_violation = true;
      }
    }
  }

  st.status = out_of_range   ? SensorStatus::INVALID
              : rate_violation ? SensorStatus::FAULTY
                               : SensorStatus::LIVE;
  st.last_arrival = std::chrono::steady_clock::now();
  st.has_arrival = true;
  // An out-of-range sample must not become the rate-of-change reference: the next good
  // sample would be differenced against the garbage spike and flagged FAULTY for nothing.
  if (!out_of_range) {
    st.prev = values;
    st.last_stamp_ns = stamp_ns;
    st.has_prev = true;
  }
}

SensorStatus SensorOverseer::status(std::size_t id) const {
  const ChannelState& st = states_[id];

  // Never received a message (also the startup state) -> not publishing.
  if (!st.has_arrival) {
    return SensorStatus::DEAD;
  }

  // Age is measured with the local steady clock, immune to per-sensor stamp clocks.
  const double age =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - st.last_arrival).count();
  if (age > specs_[id].staleness_timeout) {
    return SensorStatus::DEAD;
  }

  return st.status;
}

std::vector<SensorHealth> SensorOverseer::health() const {
  std::vector<SensorHealth> out;
  out.reserve(specs_.size());
  for (std::size_t i = 0; i < specs_.size(); ++i) {
    out.push_back({specs_[i].name, status(i)});
  }
  return out;
}
