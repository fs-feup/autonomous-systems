#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <rclcpp/time.hpp>
#include <string>
#include <vector>

/**
 * @brief Health status of a physical sensor.
 *
 * LIVE    - publishing and data passes range and rate-of-change checks.
 * FAULTY  - publishing but the latest data is suspicious (changing faster than physically
 *           possible). Its measurement rows get their noise inflated.
 * DEAD    - not publishing (no message within the staleness timeout, including startup).
 *           Its measurement rows are dropped.
 * INVALID - publishing but the latest value is physically impossible (out of range).
 *           Its measurement rows are dropped: inflating the noise is not enough for
 *           encoder glitches that are thousands of sigmas out.
 */
enum class SensorStatus : uint8_t { LIVE = 0, FAULTY = 1, DEAD = 2, INVALID = 3 };

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
 *
 * Staleness is measured with the local steady clock at message arrival, never with the
 * message header stamps: different sensors stamp with different clocks (e.g. the IMU
 * driver clock vs the vehicle ECU clock), so cross-sensor stamp comparisons are
 * meaningless. Header stamps are only compared within one sensor, for the
 * rate-of-change check.
 */
class SensorOverseer {
public:
  explicit SensorOverseer(std::vector<SensorSpec> specs);

  // Update the overseer with a new sample for the sensor @p id. The timestamp is used for the
  // rate-of-change check (same-sensor stamps only).
  void update(std::size_t id, const std::vector<double>& values, const rclcpp::Time& stamp);

  // Current status of sensor @p id, including the lazily-evaluated staleness check.
  SensorStatus status(std::size_t id) const;

  // Snapshot of every sensor's status, for telemetry publishing.
  std::vector<SensorHealth> health() const;

  std::size_t size() const { return specs_.size(); }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;

  struct ChannelState {
    std::vector<double> prev;   ///< Previous sample, for the rate-of-change check.
    int64_t last_stamp_ns = 0;  ///< Header stamp of the last sample (same-sensor clock).
    SteadyTime last_arrival;    ///< Local arrival time of the last sample, for staleness.
    bool has_prev = false;
    SensorStatus status = SensorStatus::DEAD;  ///< Latest range/rate verdict (not staleness).
  };

  std::vector<SensorSpec> specs_;
  std::vector<ChannelState> states_;
};
