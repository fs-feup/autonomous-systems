#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <algorithm>
#include <numeric>

#include "common_lib/sensor_data/imu.hpp"
#include "common_lib/sensor_data/wheel_encoders.hpp"
#include "models/observation/sensor_overseer.hpp"
#include "utils/parameters.hpp"
#include "utils/state_define.hpp"

/**
 * @brief Interface for observation models.
 *
 * A derived model builds a fixed-size measurement (z, R, h(x)). On top of that the base
 * applies sensor health, when fault detection is enabled: DEAD (not publishing) and
 * INVALID (out-of-range value) sensors have their rows dropped from the measurement,
 * FAULTY (rate-of-change violation) sensors have their rows' noise inflated by a bounded
 * factor. Independently, when fresh-measurement gating is enabled, rows of sensors that
 * delivered no new sample since the previous filter cycle are dropped, so slow sensors
 * (e.g. 6 Hz wheel encoders under a 200 Hz filter) are not re-applied as if they were
 * fresh, independent measurements. @ref row_to_sensor_ maps each measurement row to its
 * producing sensor.
 */
class ObservationModel {
protected:
  std::shared_ptr<SEParameters> parameters_;
  SensorOverseer overseer_;
  std::vector<std::size_t> row_to_sensor_;  // producing sensor id for each full measurement row

  // Per-cycle health snapshot, refreshed by get_last_observations().
  mutable std::vector<int> active_rows_;             // surviving full-row indices
  mutable std::vector<SensorStatus> sensor_status_;  // status per sensor
  mutable std::vector<bool> fresh_;  // sensor delivered a new sample since the last cycle

  // Reused scratch for the full (pre-drop) prediction, so expected_observations() does not
  // allocate on the hot path (it runs once per sigma point, every cycle).
  Eigen::VectorXd full_expected_;

  bool health_active_;  // whether the overseer runs at all (fault detection or telemetry)
  bool rows_dynamic_;   // whether rows can be dropped (fault detection or fresh gating)

  ObservationModel(const std::shared_ptr<SEParameters>& parameters, std::vector<SensorSpec> specs,
                   std::vector<std::size_t> row_to_sensor)
      : parameters_(parameters),
        overseer_(std::move(specs)),
        row_to_sensor_(std::move(row_to_sensor)),
        sensor_status_(overseer_.size(), SensorStatus::LIVE),
        fresh_(overseer_.size(), false),
        health_active_(parameters->sensor_fault_detection_enabled_ ||
                       parameters->publish_sensor_health_),
        rows_dynamic_(parameters->sensor_fault_detection_enabled_ ||
                      parameters->use_fresh_measurements_only_) {
    active_rows_.resize(row_to_sensor_.size());
    std::iota(active_rows_.begin(), active_rows_.end(), 0);
    full_expected_.resize(row_to_sensor_.size());
  }

  // Record a new sample: feed the overseer (when health tracking is enabled) and mark the
  // sensor fresh for the next cycle.
  void track(std::size_t id, const std::vector<double>& values, const rclcpp::Time& stamp) {
    if (health_active_) overseer_.update(id, values, stamp);
    fresh_[id] = true;
  }

  // Snapshot the sensor statuses and the surviving rows for this cycle, then consume the
  // fresh flags.
  void refresh_health() const {
    if (!rows_dynamic_) return;
    const bool fault_detection = parameters_->sensor_fault_detection_enabled_;
    const bool fresh_gating = parameters_->use_fresh_measurements_only_;
    if (fault_detection) {
      for (std::size_t s = 0; s < sensor_status_.size(); ++s) {
        sensor_status_[s] = overseer_.status(s);
      }
    }
    active_rows_.clear();
    for (std::size_t row = 0; row < row_to_sensor_.size(); ++row) {
      const std::size_t sensor = row_to_sensor_[row];
      if (fault_detection && (sensor_status_[sensor] == SensorStatus::DEAD ||
                              sensor_status_[sensor] == SensorStatus::INVALID)) {
        continue;
      }
      if (fresh_gating && !fresh_[sensor]) {
        continue;
      }
      active_rows_.push_back(static_cast<int>(row));
    }
    if (fresh_gating) {
      std::fill(fresh_.begin(), fresh_.end(), false);
    }
  }

  // Inflate FAULTY sensors' noise blocks and drop the inactive rows. Pass-through when the
  // rows are static, leaving the full fixed-size matrix untouched.
  Eigen::MatrixXd finalize_noise(Eigen::MatrixXd noise) const {
    if (!rows_dynamic_) return noise;
    if (parameters_->sensor_fault_detection_enabled_) {
      const double faulty_factor = parameters_->sensor_faulty_noise_factor_;
      for (std::size_t i = 0; i < row_to_sensor_.size(); ++i) {
        if (sensor_status_[row_to_sensor_[i]] != SensorStatus::FAULTY) continue;
        for (std::size_t j = 0; j < row_to_sensor_.size(); ++j) {
          if (row_to_sensor_[j] == row_to_sensor_[i]) noise(i, j) *= faulty_factor;
        }
      }
    }
    return select_active(noise);
  }

  Eigen::VectorXd select_active(const Eigen::VectorXd& full) const {
    if (active_rows_.size() == static_cast<std::size_t>(full.size())) return full;
    Eigen::VectorXd out(active_rows_.size());
    for (std::size_t k = 0; k < active_rows_.size(); ++k) out(k) = full(active_rows_[k]);
    return out;
  }

  Eigen::MatrixXd select_active(const Eigen::MatrixXd& full) const {
    if (active_rows_.size() == static_cast<std::size_t>(full.rows())) return full;
    Eigen::MatrixXd out(active_rows_.size(), active_rows_.size());
    for (std::size_t r = 0; r < active_rows_.size(); ++r) {
      for (std::size_t c = 0; c < active_rows_.size(); ++c) {
        out(r, c) = full(active_rows_[r], active_rows_[c]);
      }
    }
    return out;
  }

  // Copy the surviving rows of a full prediction into the active-sized output.
  void write_active(const Eigen::VectorXd& full, Eigen::Ref<Eigen::VectorXd> out) const {
    for (std::size_t k = 0; k < active_rows_.size(); ++k) out(k) = full(active_rows_[k]);
  }

public:
  virtual void expected_observations(const State& state,
                                     Eigen::Ref<Eigen::VectorXd> expected_observations) = 0;

  int get_measurement_size() const { return static_cast<int>(active_rows_.size()); }

  virtual Eigen::VectorXd get_last_observations() const = 0;
  virtual Eigen::MatrixXd get_last_observations_noise() const = 0;

  virtual void update_imu_data(const common_lib::sensor_data::ImuData& imu_data) = 0;
  virtual void update_wss_data(const common_lib::sensor_data::WheelEncoderData& wss_data) = 0;
  virtual void update_motor_rpm(double motor_rpm, const rclcpp::Time& stamp) = 0;
  virtual void update_steering_angle(double steering_angle, const rclcpp::Time& stamp) = 0;

  std::vector<SensorHealth> get_health() const { return overseer_.health(); }

  virtual ~ObservationModel() = default;
};
