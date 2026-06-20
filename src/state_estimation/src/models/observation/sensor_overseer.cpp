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
  // Work in raw nanoseconds so a sensor whose stamp uses a different clock source
  // (e.g. an unset RCL_SYSTEM_TIME default) can never throw on a Time subtraction.
  const int64_t now_ns = stamp.nanoseconds();

  bool faulty = false;
  const std::size_t n = std::min(values.size(), spec.signals.size());
  for (std::size_t i = 0; i < n; ++i) {
    const SignalSpec& sig = spec.signals[i];

    if (values[i] < sig.range_min || values[i] > sig.range_max) {
      faulty = true;
    }

    // Rate-of-change check is skipped on the first sample, where no prior value exists.
    if (st.has_prev && i < st.prev.size()) {
      const double dt = (now_ns - st.last_recv_ns) * 1e-9;
      if (dt > 1e-9 && std::abs(values[i] - st.prev[i]) / dt > sig.max_rate_of_change) {
        faulty = true;
      }
    }
  }

  st.status = faulty ? SensorStatus::FAULTY : SensorStatus::LIVE;
  st.prev = values;
  st.last_recv_ns = now_ns;
  st.has_prev = true;
  latest_stamp_ns_ = std::max(latest_stamp_ns_, now_ns);
}

SensorStatus SensorOverseer::status(std::size_t id) const {
  const ChannelState& st = states_[id];

  // Never received a message (also the startup state) -> not publishing.
  if (!st.has_prev) {
    return SensorStatus::DEAD;
  }

  // Age is measured against the newest sample seen on any sensor (the stream clock).
  const double age = (latest_stamp_ns_ - st.last_recv_ns) * 1e-9;
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
