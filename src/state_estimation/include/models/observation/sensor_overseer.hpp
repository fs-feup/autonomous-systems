#pragma once

#include <cstddef>
#include <cstdint>
#include <rclcpp/time.hpp>
#include <string>
#include <vector>

/**
 * @brief Health status of a physical sensor.
 *
 * LIVE   - publishing and data passes range and rate-of-change checks.
 * FAULTY - publishing but the latest data is erroneous (out of range or changing too fast).
 * DEAD   - not publishing (no message within the staleness timeout, including startup).
 *
 */
enum class SensorStatus : uint8_t { LIVE = 0, FAULTY = 1, DEAD = 2 };

// Per-signal validation bounds
struct SignalSpec {
  double range_min = -1e9;
  double range_max = 1e9;
  double max_rate_of_change = 1e12;  ///< Max accepted |d(value)/dt| between samples.
};

// Static description of a physical sensor used to construct the overseer.
struct SensorSpec {
  std::string name;
  std::vector<SignalSpec> signals;  ///< One entry per signal the sensor publishes.
  double staleness_timeout = 0.1;   ///< Seconds without a message before DEAD.
};

struct SensorHealth {
  std::string name;
  SensorStatus status;
};

/**
 * @brief Sensor health tracker. Maintains the health status of multiple physical
 * sensors based on range and rate-of-change checks and staleness timeouts.
 */
class SensorOverseer {
public:
  explicit SensorOverseer(std::vector<SensorSpec> specs);

  // Update the overseer with a new sample for the sensor @p id. The timestamp is used for staleness
  // checks.
  void update(std::size_t id, const std::vector<double>& values, const rclcpp::Time& stamp);

  // Current status of sensor @p id, including the lazily-evaluated staleness check.
  SensorStatus status(std::size_t id) const;

  // Snapshot of every sensor's status, for telemetry publishing.
  std::vector<SensorHealth> health() const;

  std::size_t size() const { return specs_.size(); }

private:
  struct ChannelState {
    std::vector<double> prev;  ///< Previous sample, for the rate-of-change check.
    int64_t last_recv_ns = 0;  ///< Stamp of the last accepted sample (ns).
    bool has_prev = false;
    SensorStatus status = SensorStatus::DEAD;  ///< Latest range/rate verdict (not staleness).
  };

  std::vector<SensorSpec> specs_;
  std::vector<ChannelState> states_;
  int64_t latest_stamp_ns_ = 0;
};
