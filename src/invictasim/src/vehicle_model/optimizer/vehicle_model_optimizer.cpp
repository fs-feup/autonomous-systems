#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <map>
#include <chrono>
#include <random>
#include <algorithm>
#include <future>
#include <filesystem>
#include <csignal>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>
#include <limits>



#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "common_lib/car_parameters/car_parameters.hpp"
#include "common_lib/structures/wheels.hpp"
#include "config/config.hpp"

// Include your high-performance inline models here!
// Make sure to rename inline_02.cpp to inline_02.hpp in your file explorer.
#include "inline_models/inline_02.hpp" 
// #include "inline_models/inline_03.hpp" // Add future models easily

// Constants
constexpr double kEpsilon = 1e-6;

// ---- Global run control: wall-clock deadline, graceful stop, convergence log ----
static std::chrono::steady_clock::time_point g_start_time;
static std::chrono::steady_clock::time_point g_deadline;
static std::atomic<bool> g_has_deadline{false};
static std::atomic<bool> g_stop{false};
static std::atomic<long> g_eval_count{0};
static std::mutex g_log_mutex;
static std::string g_convergence_log_path = "/home/ws/performance/invictasim_tuning/convergence_log.csv";

inline bool should_stop() {
    if (g_stop.load(std::memory_order_relaxed)) return true;
    if (g_has_deadline.load(std::memory_order_relaxed) &&
        std::chrono::steady_clock::now() >= g_deadline) { g_stop.store(true); return true; }
    return false;
}

void handle_sigint(int) { g_stop.store(true); }

void log_convergence(const std::string& tag, double score, double pos_rmse, double vel_rmse) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - g_start_time).count();
    std::ofstream f(g_convergence_log_path, std::ios::app);
    if (f.is_open())
        f << elapsed << ',' << g_eval_count.load() << ',' << tag << ',' << score << ','
          << pos_rmse << ',' << vel_rmse << '\n';
}

struct CsvRow {
    double timestamp_s, throttle_fl, throttle_fr, throttle_rl, throttle_rr, steering;
    double real_x, real_y, real_yaw, real_vx, real_vy, real_yaw_rate;
    double real_fl_rpm, real_fr_rpm, real_rl_rpm, real_rr_rpm, real_motor_rpm;
    bool has_rear_wheel_rpm = false;
};

struct RunningRmse {
    double sum_squares = 0.0;
    double sum_abs = 0.0;
    double max_abs = 0.0;
    int count = 0;
    void update(double value) {
        sum_squares += value * value;
        sum_abs += std::abs(value);
        max_abs = std::max(max_abs, std::abs(value));
        count++;
    }
    double get() const { return count <= 0 ? 0.0 : std::sqrt(sum_squares / static_cast<double>(count)); }
    // The acceptance criterion is stated as a *mean* error, so the search has to
    // optimise the mean rather than the RMS (which a few corners would dominate).
    double mean() const { return count <= 0 ? 0.0 : sum_abs / static_cast<double>(count); }
    double max() const { return max_abs; }
};

struct ParameterSpec { std::string name; double min_val; double max_val; };
struct EvaluationMetrics {
    double position_rmse = 1e9;
    double heading_rmse = 1e9;
    double velocity_rmse = 1e9;
    double velocity_x_rmse = 1e9;
    double velocity_y_rmse = 1e9;
    double yaw_rate_rmse = 1e9;
    double front_wheel_rpm_rmse = 1e9;
    double motor_rpm_rmse = 1e9;
    // The acceptance criterion is a mean position error and a velocity error, so
    // those are carried explicitly rather than inferred from the RMS values.
    double position_mean = 1e9;
    double velocity_mean = 1e9;
    double position_max = 1e9;
    double velocity_max = 1e9;
    double in_limit_fraction = 0.0;
};
struct Individual { std::vector<double> values; double score = -1e9; EvaluationMetrics metrics; };
struct EvaluationResult { double total_score; EvaluationMetrics metrics; };
struct PoseSample { double x; double y; double yaw; };

double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

PoseSample transform_pose_to_map(const PoseSample& pose, const PoseSample& source_origin, const PoseSample& target_origin) {
    const double dx = pose.x - source_origin.x;
    const double dy = pose.y - source_origin.y;
    const double source_cos = std::cos(source_origin.yaw);
    const double source_sin = std::sin(source_origin.yaw);
    const double local_x = source_cos * dx + source_sin * dy;
    const double local_y = -source_sin * dx + source_cos * dy;
    const double local_yaw = normalize_angle(pose.yaw - source_origin.yaw);

    const double target_cos = std::cos(target_origin.yaw);
    const double target_sin = std::sin(target_origin.yaw);
    return {target_origin.x + target_cos * local_x - target_sin * local_y,
            target_origin.y + target_sin * local_x + target_cos * local_y,
            normalize_angle(target_origin.yaw + local_yaw)};
}

// Index of a named column, or -1 when the file does not carry it.
int column_index(const std::vector<std::string>& headers, const std::string& name) {
    for (size_t i = 0; i < headers.size(); ++i) {
        if (headers[i] == name) return static_cast<int>(i);
    }
    return -1;
}

// One numeric cell, tolerating missing columns and unparseable text.
double cell_value(const std::vector<std::string>& values, int index, double fallback = 0.0) {
    if (index < 0 || index >= static_cast<int>(values.size())) return fallback;
    try {
        return std::stod(values[index]);
    } catch (...) {
        return fallback;
    }
}

// Half-width, in samples, of a filter window covering the given time span.
int half_window_samples(double seconds, double sample_dt) {
    return std::max(1, static_cast<int>(std::lround(seconds / std::max(sample_dt, 1e-6))));
}

// True when the car keeps moving for at least hold_seconds from `first` onwards.
bool moving_is_sustained(const std::vector<CsvRow>& rows, const std::vector<bool>& moving,
                         size_t first, double hold_seconds) {
    const double begin = rows[first].timestamp_s;
    for (size_t j = first; j < rows.size(); ++j) {
        if (!moving[j]) return false;
        if (rows[j].timestamp_s - begin >= hold_seconds) return true;
    }
    return false;
}

std::vector<CsvRow> read_csv(const std::string& path) {
    std::vector<CsvRow> rows;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open CSV file: " << path << std::endl;
        return rows;
    }

    std::string line;
    if (!std::getline(file, line)) return rows;

    std::vector<std::string> headers;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        while (!cell.empty() && (cell.back() == '\r' || cell.back() == ' ')) cell.pop_back();
        headers.push_back(cell);
    }

    int idx_time = column_index(headers, "timestamp_s"), idx_throttle_fl = column_index(headers, "throttle_fl");
    int idx_throttle_fr = column_index(headers, "throttle_fr"), idx_throttle_rl = column_index(headers, "throttle_rl");
    int idx_throttle_rr = column_index(headers, "throttle_rr"), idx_steering = column_index(headers, "steering");
    int idx_x = column_index(headers, "real_x"), idx_y = column_index(headers, "real_y"), idx_yaw = column_index(headers, "real_yaw");
    int idx_vx = column_index(headers, "real_vx"), idx_vy = column_index(headers, "real_vy"), idx_yaw_rate = column_index(headers, "real_yaw_rate");
    int idx_fl_rpm = column_index(headers, "real_fl_rpm"), idx_fr_rpm = column_index(headers, "real_fr_rpm");
    int idx_rl_rpm = column_index(headers, "real_rl_rpm"), idx_rr_rpm = column_index(headers, "real_rr_rpm");
    int idx_motor_rpm = column_index(headers, "real_motor_rpm");

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::vector<std::string> values;
        std::stringstream line_ss(line);
        while (std::getline(line_ss, cell, ',')) values.push_back(cell);
        if (values.size() < headers.size()) continue;

        CsvRow row;
        row.timestamp_s = cell_value(values, idx_time); row.throttle_fl = cell_value(values, idx_throttle_fl);
        row.throttle_fr = cell_value(values, idx_throttle_fr); row.throttle_rl = cell_value(values, idx_throttle_rl);
        row.throttle_rr = cell_value(values, idx_throttle_rr); row.steering = cell_value(values, idx_steering);
        row.real_x = cell_value(values, idx_x); row.real_y = cell_value(values, idx_y); row.real_yaw = cell_value(values, idx_yaw);
        row.real_vx = cell_value(values, idx_vx); row.real_vy = cell_value(values, idx_vy); row.real_yaw_rate = cell_value(values, idx_yaw_rate);
        row.real_fl_rpm = cell_value(values, idx_fl_rpm); row.real_fr_rpm = cell_value(values, idx_fr_rpm);
        row.has_rear_wheel_rpm = idx_rl_rpm >= 0 && idx_rr_rpm >= 0;
        row.real_rl_rpm = cell_value(values, idx_rl_rpm); row.real_rr_rpm = cell_value(values, idx_rr_rpm);
        row.real_motor_rpm = cell_value(values, idx_motor_rpm);
        rows.push_back(row);
    }
    return rows;
}

// The recorded reference velocity / yaw-rate come from an on-car estimator that
// is logged as a zero-order hold and contains non-physical single-sample spikes
// (apparent accelerations of thousands of m/s^2).  The true vehicle motion is
// smooth (<~10 Hz), so we condition the reference with a zero-phase, spike-
// removing filter before any error is measured against it.  Only the *reference*
// signals are filtered; driver inputs and pose are left untouched.  This is a
// ground-truth conditioning step, not a model change.
static std::vector<double> median_filter(const std::vector<double>& x, int half) {
    const int n = static_cast<int>(x.size());
    std::vector<double> out(n);
    std::vector<double> window;
    window.reserve(2 * half + 1);
    for (int i = 0; i < n; ++i) {
        window.clear();
        for (int j = std::max(0, i - half); j <= std::min(n - 1, i + half); ++j)
            window.push_back(x[j]);
        std::nth_element(window.begin(), window.begin() + window.size() / 2, window.end());
        out[i] = window[window.size() / 2];
    }
    return out;
}

static std::vector<double> moving_average(const std::vector<double>& x, int half) {
    const int n = static_cast<int>(x.size());
    std::vector<double> out(n);
    for (int i = 0; i < n; ++i) {
        double sum = 0.0; int cnt = 0;
        for (int j = std::max(0, i - half); j <= std::min(n - 1, i + half); ++j) { sum += x[j]; ++cnt; }
        out[i] = sum / std::max(1, cnt);
    }
    return out;
}

void condition_reference(std::vector<CsvRow>& rows) {
    if (rows.size() < 8) return;
    std::vector<double> vx(rows.size()), vy(rows.size()), yr(rows.size());
    for (size_t i = 0; i < rows.size(); ++i) {
        vx[i] = rows[i].real_vx; vy[i] = rows[i].real_vy; yr[i] = rows[i].real_yaw_rate;
    }
    // The estimator velocity/yaw-rate carry broadband noise of +-1.5 m/s / rad
    // at ~5-10 Hz (apparent accelerations of tens of m/s^2 that the car cannot
    // physically produce).  The true vehicle motion is smooth (<~2-3 Hz), so the
    // reference is conditioned with a spike-removing median followed by a
    // zero-phase low-pass (two moving-average passes, ~150 ms window ≈ 3 Hz).
    // At ~650 Hz sampling this preserves real accel/decel and cornering while
    // removing the noise that would otherwise breach the 2 m/s limit on its own.
    // Only the reference is filtered; commands and pose are untouched.
    // The window sizes are derived from the recording's own sample rate.  Fixed
    // sample counts made the cutoff depend on how fast the bag happened to be
    // logged -- the trackdrive merges to ~1385 Hz and the skidpad to ~965 Hz, so
    // the same constants low-passed the two references differently and the two
    // datasets were not being scored on comparable ground truth.
    std::vector<double> intervals;
    intervals.reserve(rows.size());
    for (size_t i = 1; i < rows.size(); ++i) {
        const double dt = rows[i].timestamp_s - rows[i - 1].timestamp_s;
        if (dt > 0.0) intervals.push_back(dt);
    }
    if (intervals.empty()) return;
    std::nth_element(intervals.begin(), intervals.begin() + intervals.size() / 2, intervals.end());
    const double sample_dt = intervals[intervals.size() / 2];
    const int med_half = half_window_samples(0.015, sample_dt);  // 30 ms of spike rejection
    const int ma_half = half_window_samples(0.037, sample_dt);   // 75 ms; twice -> ~3 Hz zero-phase LP
    vx = moving_average(moving_average(median_filter(vx, med_half), ma_half), ma_half);
    vy = moving_average(moving_average(median_filter(vy, med_half), ma_half), ma_half);
    yr = moving_average(moving_average(median_filter(yr, med_half), ma_half), ma_half);
    for (size_t i = 0; i < rows.size(); ++i) {
        rows[i].real_vx = vx[i]; rows[i].real_vy = vy[i]; rows[i].real_yaw_rate = yr[i];
    }
}

// The converted bags are an outer join of every topic, so they carry ~1.4 kHz of
// mostly forward-filled duplicate rows whose spacing follows message arrival, not
// time.  Scoring on them weights densely-sampled instants more heavily and makes
// every evaluation ~14x more expensive than it needs to be.  Replay is therefore
// done on a uniform grid: one row per 1/hz, holding the last measurement at or
// before each grid instant (the same zero-order hold the recording already uses).
std::vector<CsvRow> resample_rows(const std::vector<CsvRow>& rows, double hz) {
    if (hz <= 0.0 || rows.size() < 2) return rows;
    const double dt = 1.0 / hz;
    std::vector<CsvRow> out;
    out.reserve(static_cast<size_t>((rows.back().timestamp_s - rows.front().timestamp_s) * hz) + 2);
    size_t i = 0;
    for (double t = rows.front().timestamp_s; t <= rows.back().timestamp_s; t += dt) {
        while (i + 1 < rows.size() && rows[i + 1].timestamp_s <= t) ++i;
        CsvRow row = rows[i];
        row.timestamp_s = t;
        out.push_back(row);
    }
    return out;
}

std::map<std::string, double*> get_parameter_ptrs(std::shared_ptr<common_lib::car_parameters::CarParameters> p) {
  std::map<std::string, double*> m;
  if (!p) return m;

  m["car.front_bearing_drag"] = &p->front_bearing_drag;
  m["car.wheel_diameter"] = &p->wheel_diameter;
  m["car.wheelbase"] = &p->wheelbase;
  m["car.track_width"] = &p->track_width;
  m["car.cg_2_rear_axis"] = &p->cg_2_rear_axis;
  m["car.gear_ratio"] = &p->gear_ratio;
  m["car.cg_height"] = &p->cg_height;
  m["car.front_wheel_inertia"] = &p->front_wheel_inertia;
  m["car.rear_wheel_inertia"] = &p->rear_wheel_inertia;
  m["car.sprung_mass"] = &p->sprung_mass;
  m["car.unsprung_mass"] = &p->unsprung_mass;
  m["car.total_mass"] = &p->total_mass;
  m["car.sprung_cg_y"] = &p->sprung_cg_y;
  m["car.sprung_cg_z"] = &p->sprung_cg_z;
  m["car.unsprung_cg_y"] = &p->unsprung_cg_y;
  m["car.unsprung_cg_z"] = &p->unsprung_cg_z;
  m["car.Izz"] = &p->Izz;
  m["car.ackerman_factor"] = &p->ackerman_factor;

  if (p->aero_parameters) {
    m["aero.lift_coefficient"] = &p->aero_parameters->lift_coefficient;
    m["aero.drag_coefficient"] = &p->aero_parameters->drag_coefficient;
    m["aero.aero_side_force_coefficient"] = &p->aero_parameters->aero_side_force_coefficient;
    m["aero.aero_balance_front"] = &p->aero_parameters->aero_balance_front;
    m["aero.frontal_area"] = &p->aero_parameters->frontal_area;
    m["aero.air_density"] = &p->aero_parameters->air_density;
  }
  if (p->battery_parameters) {
    m["battery.capacity_ah"] = &p->battery_parameters->capacity_ah;
    m["battery.initial_soc"] = &p->battery_parameters->initial_soc;
    m["battery.max_discharge_current"] = &p->battery_parameters->max_discharge_current;
    m["battery.max_charge_current"] = &p->battery_parameters->max_charge_current;
    m["battery.min_voltage"] = &p->battery_parameters->min_voltage;
    m["battery.min_soc"] = &p->battery_parameters->min_soc;
    m["battery.OCV_a5"] = &p->battery_parameters->OCV_a5;
    m["battery.OCV_a4"] = &p->battery_parameters->OCV_a4;
    m["battery.OCV_a3"] = &p->battery_parameters->OCV_a3;
    m["battery.OCV_a2"] = &p->battery_parameters->OCV_a2;
    m["battery.OCV_a1"] = &p->battery_parameters->OCV_a1;
    m["battery.OCV_a0"] = &p->battery_parameters->OCV_a0;
    m["battery.R0_a5"] = &p->battery_parameters->R0_a5;
    m["battery.R0_a4"] = &p->battery_parameters->R0_a4;
    m["battery.R0_a3"] = &p->battery_parameters->R0_a3;
    m["battery.R0_a2"] = &p->battery_parameters->R0_a2;
    m["battery.R0_a1"] = &p->battery_parameters->R0_a1;
    m["battery.R0_a0"] = &p->battery_parameters->R0_a0;
    m["battery.R1_a5"] = &p->battery_parameters->R1_a5;
    m["battery.R1_a4"] = &p->battery_parameters->R1_a4;
    m["battery.R1_a3"] = &p->battery_parameters->R1_a3;
    m["battery.R1_a2"] = &p->battery_parameters->R1_a2;
    m["battery.R1_a1"] = &p->battery_parameters->R1_a1;
    m["battery.R1_a0"] = &p->battery_parameters->R1_a0;
    m["battery.C1_a5"] = &p->battery_parameters->C1_a5;
    m["battery.C1_a4"] = &p->battery_parameters->C1_a4;
    m["battery.C1_a3"] = &p->battery_parameters->C1_a3;
    m["battery.C1_a2"] = &p->battery_parameters->C1_a2;
    m["battery.C1_a1"] = &p->battery_parameters->C1_a1;
    m["battery.C1_a0"] = &p->battery_parameters->C1_a0;
  }
  if (p->brake_parameters) {
    m["brake.max_front_torque"] = &p->brake_parameters->max_front_torque;
    m["brake.max_rear_torque"] = &p->brake_parameters->max_rear_torque;
  }
  if (p->inverter_parameters) {
    m["inverter.efficiency"] = &p->inverter_parameters->efficiency;
    m["inverter.max_phase_current"] = &p->inverter_parameters->max_phase_current;
    m["inverter.acceleration_ramp_ms"] = &p->inverter_parameters->acceleration_ramp_ms;
    m["inverter.regen_braking_ramp_ms"] = &p->inverter_parameters->regen_braking_ramp_ms;
    m["inverter.regen_torque_fraction"] = &p->inverter_parameters->regen_torque_fraction;
    m["inverter.max_torque"] = &p->inverter_parameters->max_torque;
  }
  if (p->load_transfer_parameters) {
    m["load_transfer.roll_axis_z"] = &p->load_transfer_parameters->roll_axis_z;
    m["load_transfer.front_roll_center_z"] = &p->load_transfer_parameters->front_roll_center_z;
    m["load_transfer.rear_roll_center_z"] = &p->load_transfer_parameters->rear_roll_center_z;
    m["load_transfer.front_stiffness_distribution"] = &p->load_transfer_parameters->front_stiffness_distribution;
    m["load_transfer.pitch_center_z"] = &p->load_transfer_parameters->pitch_center_z;
  }
  if (p->motor_parameters) {
    m["motor.max_peak_rpm"] = &p->motor_parameters->max_peak_rpm;
    m["motor.max_continuous_rpm"] = &p->motor_parameters->max_continuous_rpm;
    m["motor.max_peak_current"] = &p->motor_parameters->max_peak_current;
    m["motor.max_continuous_current"] = &p->motor_parameters->max_continuous_current;
    m["motor.max_continuous_power"] = &p->motor_parameters->max_continuous_power;
    m["motor.max_peak_power"] = &p->motor_parameters->max_peak_power;
    m["motor.max_continuous_torque"] = &p->motor_parameters->max_continuous_torque;
    m["motor.max_peak_torque"] = &p->motor_parameters->max_peak_torque;
    m["motor.kt_constant"] = &p->motor_parameters->kt_constant;
    m["motor.peak_duration"] = &p->motor_parameters->peak_duration;
    m["motor.fade_start"] = &p->motor_parameters->fade_start;
  }
  if (p->steering_motor_parameters) {
    m["steering_motor.kp"] = &p->steering_motor_parameters->kp;
    m["steering_motor.ki"] = &p->steering_motor_parameters->ki;
    m["steering_motor.kd"] = &p->steering_motor_parameters->kd;
    m["steering_motor.time_constant"] = &p->steering_motor_parameters->time_constant;
  }
  if (p->steering_parameters) {
    m["steering.minimum_steering_angle"] = &p->steering_parameters->minimum_steering_angle;
    m["steering.maximum_steering_angle"] = &p->steering_parameters->maximum_steering_angle;
    m["steering.ackerman_factor"] = &p->steering_parameters->ackerman_factor;
  }
  if (p->tire_parameters) {
    m["tire.tire_B_lateral"] = &p->tire_parameters->tire_B_lateral;
    m["tire.tire_C_lateral"] = &p->tire_parameters->tire_C_lateral;
    m["tire.tire_D_lateral"] = &p->tire_parameters->tire_D_lateral;
    m["tire.tire_E_lateral"] = &p->tire_parameters->tire_E_lateral;
    m["tire.tire_B_longitudinal"] = &p->tire_parameters->tire_B_longitudinal;
    m["tire.tire_C_longitudinal"] = &p->tire_parameters->tire_C_longitudinal;
    m["tire.tire_D_longitudinal"] = &p->tire_parameters->tire_D_longitudinal;
    m["tire.tire_E_longitudinal"] = &p->tire_parameters->tire_E_longitudinal;
    m["tire.camber_scaling_factor"] = &p->tire_parameters->camber_scaling_factor;
    m["tire.effective_tire_r"] = &p->tire_parameters->effective_tire_r;
    m["tire.fr_toe"] = &p->tire_parameters->fr_toe;
    m["tire.fl_toe"] = &p->tire_parameters->fl_toe;
    m["tire.rr_toe"] = &p->tire_parameters->rr_toe;
    m["tire.rl_toe"] = &p->tire_parameters->rl_toe;
    m["tire.slip_angle_relaxation_length"] = &p->tire_parameters->slip_angle_relaxation_length;
    m["tire.slip_ratio_relaxation_length"] = &p->tire_parameters->slip_ratio_relaxation_length;
    m["tire.front_lateral_stiffness_scale"] = &p->tire_parameters->front_lateral_stiffness_scale;
    m["tire.rear_lateral_stiffness_scale"] = &p->tire_parameters->rear_lateral_stiffness_scale;
    m["tire.front_lateral_peak_scale"] = &p->tire_parameters->front_lateral_peak_scale;
    m["tire.rear_lateral_peak_scale"] = &p->tire_parameters->rear_lateral_peak_scale;
    m["tire.d_bright"] = &p->tire_parameters->d_bright;
    m["tire.d_bleft"] = &p->tire_parameters->d_bleft;
    m["tire.d_fright"] = &p->tire_parameters->d_fright;
    m["tire.d_fleft"] = &p->tire_parameters->d_fleft;
    m["tire.fr_camber"] = &p->tire_parameters->fr_camber;
    m["tire.fl_camber"] = &p->tire_parameters->fl_camber;
    m["tire.rr_camber"] = &p->tire_parameters->rr_camber;
    m["tire.rl_camber"] = &p->tire_parameters->rl_camber;
    m["tire.UNLOADED_RADIUS"] = &p->tire_parameters->UNLOADED_RADIUS;
    m["tire.WIDTH"] = &p->tire_parameters->WIDTH;
    m["tire.ASPECT_RATIO"] = &p->tire_parameters->ASPECT_RATIO;
    m["tire.RIM_RADIUS"] = &p->tire_parameters->RIM_RADIUS;
    m["tire.RIM_WIDTH"] = &p->tire_parameters->RIM_WIDTH;
    m["tire.INFLPRES"] = &p->tire_parameters->INFLPRES;
    m["tire.NOMPRES"] = &p->tire_parameters->NOMPRES;
    m["tire.MASS"] = &p->tire_parameters->MASS;
    m["tire.IXX"] = &p->tire_parameters->IXX;
    m["tire.IYY"] = &p->tire_parameters->IYY;
    m["tire.BELT_MASS"] = &p->tire_parameters->BELT_MASS;
    m["tire.BELT_IXX"] = &p->tire_parameters->BELT_IXX;
    m["tire.BELT_IYY"] = &p->tire_parameters->BELT_IYY;
    m["tire.GRAVITY"] = &p->tire_parameters->GRAVITY;
    m["tire.FNOMIN"] = &p->tire_parameters->FNOMIN;
    m["tire.VERTICAL_STIFFNESS"] = &p->tire_parameters->VERTICAL_STIFFNESS;
    m["tire.VERTICAL_DAMPING"] = &p->tire_parameters->VERTICAL_DAMPING;
    m["tire.MC_CONTOUR_A"] = &p->tire_parameters->MC_CONTOUR_A;
    m["tire.MC_CONTOUR_B"] = &p->tire_parameters->MC_CONTOUR_B;
    m["tire.BREFF"] = &p->tire_parameters->BREFF;
    m["tire.DREFF"] = &p->tire_parameters->DREFF;
    m["tire.FREFF"] = &p->tire_parameters->FREFF;
    m["tire.Q_RE0"] = &p->tire_parameters->Q_RE0;
    m["tire.Q_V1"] = &p->tire_parameters->Q_V1;
    m["tire.Q_V2"] = &p->tire_parameters->Q_V2;
    m["tire.Q_FZ2"] = &p->tire_parameters->Q_FZ2;
    m["tire.Q_FCX"] = &p->tire_parameters->Q_FCX;
    m["tire.Q_FCY"] = &p->tire_parameters->Q_FCY;
    m["tire.Q_CAM"] = &p->tire_parameters->Q_CAM;
    m["tire.PFZ1"] = &p->tire_parameters->PFZ1;
    m["tire.Q_FCY2"] = &p->tire_parameters->Q_FCY2;
    m["tire.Q_CAM1"] = &p->tire_parameters->Q_CAM1;
    m["tire.Q_CAM2"] = &p->tire_parameters->Q_CAM2;
    m["tire.Q_CAM3"] = &p->tire_parameters->Q_CAM3;
    m["tire.Q_FYS1"] = &p->tire_parameters->Q_FYS1;
    m["tire.Q_FYS2"] = &p->tire_parameters->Q_FYS2;
    m["tire.Q_FYS3"] = &p->tire_parameters->Q_FYS3;
    m["tire.BOTTOM_OFFST"] = &p->tire_parameters->BOTTOM_OFFST;
    m["tire.BOTTOM_STIFF"] = &p->tire_parameters->BOTTOM_STIFF;
    m["tire.LONGITUDINAL_STIFFNESS"] = &p->tire_parameters->LONGITUDINAL_STIFFNESS;
    m["tire.LATERAL_STIFFNESS"] = &p->tire_parameters->LATERAL_STIFFNESS;
    m["tire.YAW_STIFFNESS"] = &p->tire_parameters->YAW_STIFFNESS;
    m["tire.FREQ_LONG"] = &p->tire_parameters->FREQ_LONG;
    m["tire.FREQ_LAT"] = &p->tire_parameters->FREQ_LAT;
    m["tire.FREQ_YAW"] = &p->tire_parameters->FREQ_YAW;
    m["tire.FREQ_WINDUP"] = &p->tire_parameters->FREQ_WINDUP;
    m["tire.DAMP_LONG"] = &p->tire_parameters->DAMP_LONG;
    m["tire.DAMP_LAT"] = &p->tire_parameters->DAMP_LAT;
    m["tire.DAMP_YAW"] = &p->tire_parameters->DAMP_YAW;
    m["tire.DAMP_WINDUP"] = &p->tire_parameters->DAMP_WINDUP;
    m["tire.DAMP_RESIDUAL"] = &p->tire_parameters->DAMP_RESIDUAL;
    m["tire.DAMP_VLOW"] = &p->tire_parameters->DAMP_VLOW;
    m["tire.Q_BVX"] = &p->tire_parameters->Q_BVX;
    m["tire.Q_BVT"] = &p->tire_parameters->Q_BVT;
    m["tire.PCFX1"] = &p->tire_parameters->PCFX1;
    m["tire.PCFX2"] = &p->tire_parameters->PCFX2;
    m["tire.PCFX3"] = &p->tire_parameters->PCFX3;
    m["tire.PCFY1"] = &p->tire_parameters->PCFY1;
    m["tire.PCFY2"] = &p->tire_parameters->PCFY2;
    m["tire.PCFY3"] = &p->tire_parameters->PCFY3;
    m["tire.PCMZ1"] = &p->tire_parameters->PCMZ1;
    m["tire.Q_RA1"] = &p->tire_parameters->Q_RA1;
    m["tire.Q_RA2"] = &p->tire_parameters->Q_RA2;
    m["tire.Q_RB1"] = &p->tire_parameters->Q_RB1;
    m["tire.Q_RB2"] = &p->tire_parameters->Q_RB2;
    m["tire.ELLIPS_SHIFT"] = &p->tire_parameters->ELLIPS_SHIFT;
    m["tire.ELLIPS_LENGTH"] = &p->tire_parameters->ELLIPS_LENGTH;
    m["tire.ELLIPS_HEIGHT"] = &p->tire_parameters->ELLIPS_HEIGHT;
    m["tire.ELLIPS_ORDER"] = &p->tire_parameters->ELLIPS_ORDER;
    m["tire.ELLIPS_MAX_STEP"] = &p->tire_parameters->ELLIPS_MAX_STEP;
    m["tire.ELLIPS_NWIDTH"] = &p->tire_parameters->ELLIPS_NWIDTH;
    m["tire.ELLIPS_NLENGTH"] = &p->tire_parameters->ELLIPS_NLENGTH;
    m["tire.PRESMIN"] = &p->tire_parameters->PRESMIN;
    m["tire.PRESMAX"] = &p->tire_parameters->PRESMAX;
    m["tire.FZMIN"] = &p->tire_parameters->FZMIN;
    m["tire.FZMAX"] = &p->tire_parameters->FZMAX;
    m["tire.KPUMIN"] = &p->tire_parameters->KPUMIN;
    m["tire.KPUMAX"] = &p->tire_parameters->KPUMAX;
    m["tire.ALPMIN"] = &p->tire_parameters->ALPMIN;
    m["tire.ALPMAX"] = &p->tire_parameters->ALPMAX;
    m["tire.CAMMIN"] = &p->tire_parameters->CAMMIN;
    m["tire.CAMMAX"] = &p->tire_parameters->CAMMAX;
    m["tire.LFZO"] = &p->tire_parameters->LFZO;
    m["tire.LCX"] = &p->tire_parameters->LCX;
    m["tire.LMUX"] = &p->tire_parameters->LMUX;
    m["tire.LEX"] = &p->tire_parameters->LEX;
    m["tire.LKX"] = &p->tire_parameters->LKX;
    m["tire.LHX"] = &p->tire_parameters->LHX;
    m["tire.LVX"] = &p->tire_parameters->LVX;
    m["tire.LCY"] = &p->tire_parameters->LCY;
    m["tire.LMUY"] = &p->tire_parameters->LMUY;
    m["tire.LEY"] = &p->tire_parameters->LEY;
    m["tire.LKY"] = &p->tire_parameters->LKY;
    m["tire.LHY"] = &p->tire_parameters->LHY;
    m["tire.LVY"] = &p->tire_parameters->LVY;
    m["tire.LTR"] = &p->tire_parameters->LTR;
    m["tire.LRES"] = &p->tire_parameters->LRES;
    m["tire.LXAL"] = &p->tire_parameters->LXAL;
    m["tire.LYKA"] = &p->tire_parameters->LYKA;
    m["tire.LVYKA"] = &p->tire_parameters->LVYKA;
    m["tire.LS"] = &p->tire_parameters->LS;
    m["tire.LKYC"] = &p->tire_parameters->LKYC;
    m["tire.LKZC"] = &p->tire_parameters->LKZC;
    m["tire.LVMX"] = &p->tire_parameters->LVMX;
    m["tire.LMX"] = &p->tire_parameters->LMX;
    m["tire.LMY"] = &p->tire_parameters->LMY;
    m["tire.LMP"] = &p->tire_parameters->LMP;
    m["tire.PCX1"] = &p->tire_parameters->PCX1;
    m["tire.PDX1"] = &p->tire_parameters->PDX1;
    m["tire.PDX2"] = &p->tire_parameters->PDX2;
    m["tire.PDX3"] = &p->tire_parameters->PDX3;
    m["tire.PEX1"] = &p->tire_parameters->PEX1;
    m["tire.PEX2"] = &p->tire_parameters->PEX2;
    m["tire.PEX3"] = &p->tire_parameters->PEX3;
    m["tire.PEX4"] = &p->tire_parameters->PEX4;
    m["tire.PKX1"] = &p->tire_parameters->PKX1;
    m["tire.PKX2"] = &p->tire_parameters->PKX2;
    m["tire.PKX3"] = &p->tire_parameters->PKX3;
    m["tire.PHX1"] = &p->tire_parameters->PHX1;
    m["tire.PHX2"] = &p->tire_parameters->PHX2;
    m["tire.PVX1"] = &p->tire_parameters->PVX1;
    m["tire.PVX2"] = &p->tire_parameters->PVX2;
    m["tire.PPX1"] = &p->tire_parameters->PPX1;
    m["tire.PPX2"] = &p->tire_parameters->PPX2;
    m["tire.PPX3"] = &p->tire_parameters->PPX3;
    m["tire.PPX4"] = &p->tire_parameters->PPX4;
    m["tire.RBX1"] = &p->tire_parameters->RBX1;
    m["tire.RBX2"] = &p->tire_parameters->RBX2;
    m["tire.RBX3"] = &p->tire_parameters->RBX3;
    m["tire.RCX1"] = &p->tire_parameters->RCX1;
    m["tire.REX1"] = &p->tire_parameters->REX1;
    m["tire.REX2"] = &p->tire_parameters->REX2;
    m["tire.RHX1"] = &p->tire_parameters->RHX1;
    m["tire.QSX1"] = &p->tire_parameters->QSX1;
    m["tire.QSX2"] = &p->tire_parameters->QSX2;
    m["tire.QSX3"] = &p->tire_parameters->QSX3;
    m["tire.QSX4"] = &p->tire_parameters->QSX4;
    m["tire.QSX5"] = &p->tire_parameters->QSX5;
    m["tire.QSX6"] = &p->tire_parameters->QSX6;
    m["tire.QSX7"] = &p->tire_parameters->QSX7;
    m["tire.QSX8"] = &p->tire_parameters->QSX8;
    m["tire.QSX9"] = &p->tire_parameters->QSX9;
    m["tire.QSX10"] = &p->tire_parameters->QSX10;
    m["tire.QSX11"] = &p->tire_parameters->QSX11;
    m["tire.QSX12"] = &p->tire_parameters->QSX12;
    m["tire.QSX13"] = &p->tire_parameters->QSX13;
    m["tire.QSX14"] = &p->tire_parameters->QSX14;
    m["tire.PPMX1"] = &p->tire_parameters->PPMX1;
    m["tire.PCY1"] = &p->tire_parameters->PCY1;
    m["tire.PDY1"] = &p->tire_parameters->PDY1;
    m["tire.PDY2"] = &p->tire_parameters->PDY2;
    m["tire.PDY3"] = &p->tire_parameters->PDY3;
    m["tire.PEY1"] = &p->tire_parameters->PEY1;
    m["tire.PEY2"] = &p->tire_parameters->PEY2;
    m["tire.PEY3"] = &p->tire_parameters->PEY3;
    m["tire.PEY4"] = &p->tire_parameters->PEY4;
    m["tire.PEY5"] = &p->tire_parameters->PEY5;
    m["tire.PKY1"] = &p->tire_parameters->PKY1;
    m["tire.PKY2"] = &p->tire_parameters->PKY2;
    m["tire.PKY3"] = &p->tire_parameters->PKY3;
    m["tire.PKY4"] = &p->tire_parameters->PKY4;
    m["tire.PKY5"] = &p->tire_parameters->PKY5;
    m["tire.PKY6"] = &p->tire_parameters->PKY6;
    m["tire.PKY7"] = &p->tire_parameters->PKY7;
    m["tire.PHY1"] = &p->tire_parameters->PHY1;
    m["tire.PHY2"] = &p->tire_parameters->PHY2;
    m["tire.PVY1"] = &p->tire_parameters->PVY1;
    m["tire.PVY2"] = &p->tire_parameters->PVY2;
    m["tire.PVY3"] = &p->tire_parameters->PVY3;
    m["tire.PVY4"] = &p->tire_parameters->PVY4;
    m["tire.PPY1"] = &p->tire_parameters->PPY1;
    m["tire.PPY2"] = &p->tire_parameters->PPY2;
    m["tire.PPY3"] = &p->tire_parameters->PPY3;
    m["tire.PPY4"] = &p->tire_parameters->PPY4;
    m["tire.PPY5"] = &p->tire_parameters->PPY5;
    m["tire.RBY1"] = &p->tire_parameters->RBY1;
    m["tire.RBY2"] = &p->tire_parameters->RBY2;
    m["tire.RBY3"] = &p->tire_parameters->RBY3;
    m["tire.RBY4"] = &p->tire_parameters->RBY4;
    m["tire.RCY1"] = &p->tire_parameters->RCY1;
    m["tire.REY1"] = &p->tire_parameters->REY1;
    m["tire.REY2"] = &p->tire_parameters->REY2;
    m["tire.RHY1"] = &p->tire_parameters->RHY1;
    m["tire.RHY2"] = &p->tire_parameters->RHY2;
    m["tire.RVY1"] = &p->tire_parameters->RVY1;
    m["tire.RVY2"] = &p->tire_parameters->RVY2;
    m["tire.RVY3"] = &p->tire_parameters->RVY3;
    m["tire.RVY4"] = &p->tire_parameters->RVY4;
    m["tire.RVY5"] = &p->tire_parameters->RVY5;
    m["tire.RVY6"] = &p->tire_parameters->RVY6;
    m["tire.QSY1"] = &p->tire_parameters->QSY1;
    m["tire.QSY2"] = &p->tire_parameters->QSY2;
    m["tire.QSY3"] = &p->tire_parameters->QSY3;
    m["tire.QSY4"] = &p->tire_parameters->QSY4;
    m["tire.QSY5"] = &p->tire_parameters->QSY5;
    m["tire.QSY6"] = &p->tire_parameters->QSY6;
    m["tire.QSY7"] = &p->tire_parameters->QSY7;
    m["tire.QSY8"] = &p->tire_parameters->QSY8;
    m["tire.QBZ1"] = &p->tire_parameters->QBZ1;
    m["tire.QBZ2"] = &p->tire_parameters->QBZ2;
    m["tire.QBZ3"] = &p->tire_parameters->QBZ3;
    m["tire.QBZ4"] = &p->tire_parameters->QBZ4;
    m["tire.QBZ5"] = &p->tire_parameters->QBZ5;
    m["tire.QBZ9"] = &p->tire_parameters->QBZ9;
    m["tire.QBZ10"] = &p->tire_parameters->QBZ10;
    m["tire.QCZ1"] = &p->tire_parameters->QCZ1;
    m["tire.QDZ1"] = &p->tire_parameters->QDZ1;
    m["tire.QDZ2"] = &p->tire_parameters->QDZ2;
    m["tire.QDZ3"] = &p->tire_parameters->QDZ3;
    m["tire.QDZ4"] = &p->tire_parameters->QDZ4;
    m["tire.QDZ6"] = &p->tire_parameters->QDZ6;
    m["tire.QDZ7"] = &p->tire_parameters->QDZ7;
    m["tire.QDZ8"] = &p->tire_parameters->QDZ8;
    m["tire.QDZ9"] = &p->tire_parameters->QDZ9;
    m["tire.QDZ10"] = &p->tire_parameters->QDZ10;
    m["tire.QDZ11"] = &p->tire_parameters->QDZ11;
    m["tire.QEZ1"] = &p->tire_parameters->QEZ1;
    m["tire.QEZ2"] = &p->tire_parameters->QEZ2;
    m["tire.QEZ3"] = &p->tire_parameters->QEZ3;
    m["tire.QEZ4"] = &p->tire_parameters->QEZ4;
    m["tire.QEZ5"] = &p->tire_parameters->QEZ5;
    m["tire.QHZ1"] = &p->tire_parameters->QHZ1;
    m["tire.QHZ2"] = &p->tire_parameters->QHZ2;
    m["tire.QHZ3"] = &p->tire_parameters->QHZ3;
    m["tire.QHZ4"] = &p->tire_parameters->QHZ4;
    m["tire.PPZ1"] = &p->tire_parameters->PPZ1;
    m["tire.PPZ2"] = &p->tire_parameters->PPZ2;
    m["tire.SSZ1"] = &p->tire_parameters->SSZ1;
    m["tire.SSZ2"] = &p->tire_parameters->SSZ2;
    m["tire.SSZ3"] = &p->tire_parameters->SSZ3;
    m["tire.SSZ4"] = &p->tire_parameters->SSZ4;
    m["tire.PECP1"] = &p->tire_parameters->PECP1;
    m["tire.PECP2"] = &p->tire_parameters->PECP2;
    m["tire.PDXP1"] = &p->tire_parameters->PDXP1;
    m["tire.PDXP2"] = &p->tire_parameters->PDXP2;
    m["tire.PDXP3"] = &p->tire_parameters->PDXP3;
    m["tire.PDXP4"] = &p->tire_parameters->PDXP4;
    m["tire.PDYP1"] = &p->tire_parameters->PDYP1;
    m["tire.PDYP2"] = &p->tire_parameters->PDYP2;
    m["tire.PDYP3"] = &p->tire_parameters->PDYP3;
    m["tire.PDYP4"] = &p->tire_parameters->PDYP4;
    m["tire.PKYP1"] = &p->tire_parameters->PKYP1;
    m["tire.PHYP1"] = &p->tire_parameters->PHYP1;
    m["tire.PHYP2"] = &p->tire_parameters->PHYP2;
    m["tire.PHYP3"] = &p->tire_parameters->PHYP3;
    m["tire.PHYP4"] = &p->tire_parameters->PHYP4;
    m["tire.QDTP1"] = &p->tire_parameters->QDTP1;
    m["tire.QBRP1"] = &p->tire_parameters->QBRP1;
    m["tire.QCRP1"] = &p->tire_parameters->QCRP1;
    m["tire.QCRP2"] = &p->tire_parameters->QCRP2;
    m["tire.QDRP1"] = &p->tire_parameters->QDRP1;
    m["tire.QDRP2"] = &p->tire_parameters->QDRP2;
    m["tire.PA1"] = &p->tire_parameters->PA1;
    m["tire.PA2"] = &p->tire_parameters->PA2;
    m["tire.PB1"] = &p->tire_parameters->PB1;
    m["tire.PB2"] = &p->tire_parameters->PB2;
    m["tire.PB3"] = &p->tire_parameters->PB3;
    m["tire.PAE"] = &p->tire_parameters->PAE;
    m["tire.PBE"] = &p->tire_parameters->PBE;
    m["tire.PCE"] = &p->tire_parameters->PCE;
    m["tire.PLS"] = &p->tire_parameters->PLS;
    m["tire.PW1"] = &p->tire_parameters->PW1;
    m["tire.PW2"] = &p->tire_parameters->PW2;
    m["tire.PW3"] = &p->tire_parameters->PW3;
    m["tire.PW4"] = &p->tire_parameters->PW4;
    m["tire.N_WIDTH"] = &p->tire_parameters->N_WIDTH;
    m["tire.N_WIDTH_HP"] = &p->tire_parameters->N_WIDTH_HP;
    m["tire.N_LENGTH"] = &p->tire_parameters->N_LENGTH;
    m["tire.N_LENGTH_HP"] = &p->tire_parameters->N_LENGTH_HP;
    m["tire.ROAD_SPACING"] = &p->tire_parameters->ROAD_SPACING;
    m["tire.ROAD_SPACING_HP"] = &p->tire_parameters->ROAD_SPACING_HP;
    m["tire.MAX_HEIGHT"] = &p->tire_parameters->MAX_HEIGHT;
    m["tire.CONTACT_THREADS"] = &p->tire_parameters->CONTACT_THREADS;
    m["tire.CONTACT_THREADS_HP"] = &p->tire_parameters->CONTACT_THREADS_HP;
    m["tire.EPSNL"] = &p->tire_parameters->EPSNL;
    m["tire.MC"] = &p->tire_parameters->MC;
    m["tire.IC"] = &p->tire_parameters->IC;
    m["tire.KX"] = &p->tire_parameters->KX;
    m["tire.KY"] = &p->tire_parameters->KY;
    m["tire.KP"] = &p->tire_parameters->KP;
    m["tire.CX"] = &p->tire_parameters->CX;
    m["tire.CY"] = &p->tire_parameters->CY;
    m["tire.CP"] = &p->tire_parameters->CP;
    m["tire.EP"] = &p->tire_parameters->EP;
    m["tire.EP12"] = &p->tire_parameters->EP12;
    m["tire.BF2"] = &p->tire_parameters->BF2;
    m["tire.BP1"] = &p->tire_parameters->BP1;
    m["tire.BP2"] = &p->tire_parameters->BP2;
    m["tire.BP3"] = &p->tire_parameters->BP3;
    m["tire.BP4"] = &p->tire_parameters->BP4;
    m["tire.CXZ1"] = &p->tire_parameters->CXZ1;
    m["tire.CXZ2"] = &p->tire_parameters->CXZ2;
    m["tire.CXP1"] = &p->tire_parameters->CXP1;
    m["tire.CXX1"] = &p->tire_parameters->CXX1;
    m["tire.CYZ1"] = &p->tire_parameters->CYZ1;
    m["tire.CYZ2"] = &p->tire_parameters->CYZ2;
    m["tire.CYP1"] = &p->tire_parameters->CYP1;
    m["tire.CYY1"] = &p->tire_parameters->CYY1;
    m["tire.CPZ1"] = &p->tire_parameters->CPZ1;
    m["tire.TYRE_MASS"] = &p->tire_parameters->TYRE_MASS;
    m["tire.QMB"] = &p->tire_parameters->QMB;
    m["tire.QMC"] = &p->tire_parameters->QMC;
    m["tire.QIBY"] = &p->tire_parameters->QIBY;
    m["tire.QIBXZ"] = &p->tire_parameters->QIBXZ;
    m["tire.QIC"] = &p->tire_parameters->QIC;
    m["tire.QCBXZ"] = &p->tire_parameters->QCBXZ;
    m["tire.QCBY"] = &p->tire_parameters->QCBY;
    m["tire.QCBTH"] = &p->tire_parameters->QCBTH;
    m["tire.QCBGM"] = &p->tire_parameters->QCBGM;
    m["tire.QKBXZ"] = &p->tire_parameters->QKBXZ;
    m["tire.QKBY"] = &p->tire_parameters->QKBY;
    m["tire.QKBTH"] = &p->tire_parameters->QKBTH;
    m["tire.QKBGM"] = &p->tire_parameters->QKBGM;
    m["tire.QCCX"] = &p->tire_parameters->QCCX;
    m["tire.QCCY"] = &p->tire_parameters->QCCY;
    m["tire.QCCFI"] = &p->tire_parameters->QCCFI;
    m["tire.QKBX"] = &p->tire_parameters->QKBX;
    m["tire.QKCY"] = &p->tire_parameters->QKCY;
    m["tire.QKCFI"] = &p->tire_parameters->QKCFI;
    m["tire.QBVXZ"] = &p->tire_parameters->QBVXZ;
    m["tire.QBVTH"] = &p->tire_parameters->QBVTH;
    m["tire.QRE0"] = &p->tire_parameters->QRE0;
    m["tire.QV1"] = &p->tire_parameters->QV1;
    m["tire.QV2"] = &p->tire_parameters->QV2;
    m["tire.QFCX1"] = &p->tire_parameters->QFCX1;
    m["tire.QFCY1"] = &p->tire_parameters->QFCY1;
    m["tire.QFCG1"] = &p->tire_parameters->QFCG1;
    m["tire.QFZ1"] = &p->tire_parameters->QFZ1;
    m["tire.QFZ2"] = &p->tire_parameters->QFZ2;
    m["tire.QFZ3"] = &p->tire_parameters->QFZ3;
    m["tire.QPFZ1"] = &p->tire_parameters->QPFZ1;
    m["tire.Amu"] = &p->tire_parameters->Amu;

    // Aliases for tuning configs
    m["tire.longitudinal_peak_pdx1"] = &p->tire_parameters->PDX1;
    m["tire.longitudinal_stiffness_pkx1"] = &p->tire_parameters->PKX1;
    m["tire.longitudinal_curvature_pex1"] = &p->tire_parameters->PEX1;
    m["tire.lateral_peak_pdy1"] = &p->tire_parameters->PDY1;
    m["tire.lateral_stiffness_pky1"] = &p->tire_parameters->PKY1;
    m["tire.lateral_curvature_pey1"] = &p->tire_parameters->PEY1;
    m["tire.structural_longitudinal_stiffness"] = &p->tire_parameters->LONGITUDINAL_STIFFNESS;
    m["tire.structural_lateral_stiffness"] = &p->tire_parameters->LATERAL_STIFFNESS;
    m["tire.structural_yaw_stiffness"] = &p->tire_parameters->YAW_STIFFNESS;
    m["tire.longitudinal_peak_scale"] = &p->tire_parameters->LMUX;
    m["tire.longitudinal_stiffness_scale"] = &p->tire_parameters->LKX;
    m["tire.lateral_shape_scale"] = &p->tire_parameters->LCY;
    m["tire.lateral_peak_scale"] = &p->tire_parameters->LMUY;
    m["tire.lateral_curvature_scale"] = &p->tire_parameters->LEY;
    m["tire.lateral_stiffness_scale"] = &p->tire_parameters->LKY;
    m["tire.combined_slip_lateral_stiffness_scale"] = &p->tire_parameters->LYKA;
  }
  if (p->transmission_parameters) {
    m["transmission.gear_ratio"] = &p->transmission_parameters->gear_ratio;
    m["transmission.efficiency"] = &p->transmission_parameters->efficiency;
    m["transmission.kv"] = &p->transmission_parameters->kv;
    m["transmission.viscous_drag_coeff"] = &p->transmission_parameters->viscous_drag_coeff;
    m["transmission.coulomb_drag"] = &p->transmission_parameters->coulomb_drag;
    m["transmission.coulomb_smooth_stiffness"] = &p->transmission_parameters->coulomb_smooth_stiffness;
    m["transmission.preload"] = &p->transmission_parameters->preload;
    m["transmission.drive_ramp_effect"] = &p->transmission_parameters->drive_ramp_effect;
    m["transmission.coast_ramp_effect"] = &p->transmission_parameters->coast_ramp_effect;
  }

  return m;
}


void apply_parameter_override(std::shared_ptr<common_lib::car_parameters::CarParameters> car_params, const std::string& name, double val) {
    auto ptrs = get_parameter_ptrs(car_params);
    auto it = ptrs.find(name);
    if (it != ptrs.end()) {
        *(it->second) = val;
    } else {
        std::cerr << "Warning: Unknown parameter override '" << name << "'" << std::endl;
    }
}

double get_baseline_value(std::shared_ptr<common_lib::car_parameters::CarParameters> car_params, const std::string& name) {
    auto ptrs = get_parameter_ptrs(car_params);
    auto it = ptrs.find(name);
    return (it != ptrs.end()) ? *(it->second) : 0.0;
}

std::shared_ptr<common_lib::car_parameters::CarParameters> deep_copy_car_parameters(std::shared_ptr<common_lib::car_parameters::CarParameters> src) {
    if (!src) return nullptr;
    auto dst = std::make_shared<common_lib::car_parameters::CarParameters>(*src);
    if (src->aero_parameters) dst->aero_parameters = std::make_shared<common_lib::car_parameters::AeroParameters>(*src->aero_parameters);
    if (src->tire_parameters) dst->tire_parameters = std::make_shared<common_lib::car_parameters::TireParameters>(*src->tire_parameters);
    if (src->motor_parameters) dst->motor_parameters = std::make_shared<common_lib::car_parameters::MotorParameters>(*src->motor_parameters);
    if (src->battery_parameters) dst->battery_parameters = std::make_shared<common_lib::car_parameters::BatteryParameters>(*src->battery_parameters);
    if (src->transmission_parameters) dst->transmission_parameters = std::make_shared<common_lib::car_parameters::TransmissionParameters>(*src->transmission_parameters);
    if (src->inverter_parameters) dst->inverter_parameters = std::make_shared<common_lib::car_parameters::InverterParameters>(*src->inverter_parameters);
    if (src->brake_parameters) dst->brake_parameters = std::make_shared<common_lib::car_parameters::BrakeParameters>(*src->brake_parameters);
    if (src->steering_parameters) dst->steering_parameters = std::make_shared<common_lib::car_parameters::SteeringParameters>(*src->steering_parameters);
    if (src->steering_motor_parameters) dst->steering_motor_parameters = std::make_shared<common_lib::car_parameters::SteeringMotorParameters>(*src->steering_motor_parameters);
    if (src->load_transfer_parameters) dst->load_transfer_parameters = std::make_shared<common_lib::car_parameters::LoadTransferParameters>(*src->load_transfer_parameters);
    if (src->physical_constants) dst->physical_constants = std::make_shared<common_lib::structures::PhysicalConstants>(*src->physical_constants);
    return dst;
}

// The converted bags contain front-wheel and motor RPM but no rear-wheel RPM.
// Seeding the driven wheels at zero creates an artificial launch transient and
// invalidates every candidate's first seconds.  A single-motor differential
// implies mean rear-wheel speed = motor speed / final-drive ratio.
void initialise_wheel_speeds(SimState& state, const CsvRow& sample,
                             const common_lib::car_parameters::CarParameters& params) {
    constexpr double kRpmToRadPerSec = 2.0 * M_PI / 60.0;
    state.wheels_speed.front_left = sample.real_fl_rpm * kRpmToRadPerSec;
    state.wheels_speed.front_right = sample.real_fr_rpm * kRpmToRadPerSec;
    if (sample.has_rear_wheel_rpm) {
        state.wheels_speed.rear_left = sample.real_rl_rpm * kRpmToRadPerSec;
        state.wheels_speed.rear_right = sample.real_rr_rpm * kRpmToRadPerSec;
    } else {
        const double ratio = std::max(params.transmission_parameters->gear_ratio, 1e-6);
        const double rear_omega = sample.real_motor_rpm * kRpmToRadPerSec / ratio;
        state.wheels_speed.rear_left = rear_omega;
        state.wheels_speed.rear_right = rear_omega;
    }
    state.motor_omega = sample.real_motor_rpm * kRpmToRadPerSec;
}

struct DatasetConfig {
    double weight = 1.0;
    double start_offset_s = 0.0;
    double stop_after_s = -1.0;
    double max_distance_error = 2.5;
    double max_velocity_error = 5.0;
    bool ignore_lateral_distance = false;
    // Every recording starts at a different state of charge.  The DC-bus voltage
    // at rest is the pack open-circuit voltage, which inverts through the OCV
    // polynomial to the SOC the battery model must start from; otherwise the
    // model's available power (and therefore its torque limit) is wrong.
    // Negative means "leave the configured value alone".
    double initial_soc = -1.0;
    // Open-loop position error is the double integral of the model error, so what
    // actually drives it over a long run is not the RMS error but the *signed
    // mean* of the yaw-rate and velocity errors: a steady 0.005 rad/s yaw-rate
    // bias is 0.7 rad of heading after 135 s, which no amount of low RMS can undo.
    // These weights let the search trade instantaneous accuracy for zero drift.
    double bias_yaw_rate_weight = 0.0;
    double bias_velocity_weight = 0.0;
    double heading_weight = 0.25;
    double yaw_rate_weight = 0.25;
    double limit_weight = 0.50;
    // Re-anchored scoring period (s).  >0 resets the sim to the measured state
    // every period, so the fitness rewards models that track the *whole* run in
    // short open-loop horizons rather than ones that merely survive the easy
    // opening while being globally unfaithful (e.g. directionally unstable).
    // 0 keeps the classic continuous time-to-first-failure behaviour.
    double reanchor_period = 0.0;
    // Derived from telemetry; never supplied as a hand-tuned timestamp.
    size_t start_index = 0;
    size_t end_index = std::numeric_limits<size_t>::max();
};

// Return the contiguous useful driving portion of a recording.  Velocity alone
// is deliberately cross-checked against pose increments: estimator noise after
// a bag ends otherwise looks like movement. Start requires sustained movement;
// end requires both sustained rest and inactive driver inputs.
std::pair<size_t, size_t> detect_motion_window(const std::vector<CsvRow>& rows) {
    if (rows.empty()) return {0, 0};
    // The SLAM pose jitters by ~0.5 m peak-to-peak even with the car parked, so
    // motion is judged from the net displacement over a full second against a
    // threshold well above that jitter.  The car never drives below ~5 m/s in
    // these recordings, so this cannot clip real driving.
    constexpr double kMovingSpeed = 1.0;
    constexpr double kActiveInput = 0.02;
    constexpr double kHoldSeconds = 0.50;
    std::vector<bool> moving(rows.size(), false);
    std::vector<bool> input_active(rows.size(), false);
    // Motion is judged from the *pose*, not from the estimator velocity.  When a
    // run ends the velocity topic can simply stop publishing, and the forward-fill
    // in the converter then holds its last value forever -- Skidpad 12 sits at a
    // stale 5.1 m/s for its final five seconds while the car is parked.  Taking
    // the max of the two signals (as before) made that tail look like driving and
    // pulled five seconds of pure nonsense into the scoring window.  Displacement
    // is measured over a one-second span so SLAM jitter cannot fake motion either.
    constexpr double kPoseSpan = 1.0;
    for (size_t i = 0; i < rows.size(); ++i) {
        const auto& r = rows[i];
        size_t j = i;
        while (j + 1 < rows.size() && rows[j + 1].timestamp_s - r.timestamp_s < kPoseSpan) ++j;
        double pose_speed = 0.0;
        const double span = rows[j].timestamp_s - r.timestamp_s;
        // Near the end of the recording the span collapses, and dividing jitter by
        // a near-zero span would read as motion; require most of a full span.
        if (span > 0.9 * kPoseSpan)
            pose_speed = std::hypot(rows[j].real_x - r.real_x, rows[j].real_y - r.real_y) / span;
        const double drive = std::max(std::abs(r.throttle_rl), std::abs(r.throttle_rr));
        moving[i] = pose_speed >= kMovingSpeed;
        input_active[i] = drive >= kActiveInput || std::abs(r.steering) >= kActiveInput;
    }
    size_t start = 0;
    while (start + 1 < rows.size() && !moving_is_sustained(rows, moving, start, kHoldSeconds)) {
        ++start;
    }
    // End at the last instant the car was actually moving.  Driver input cannot be
    // used to extend the window: after a run the last command is held forever by
    // the converter's forward fill (Skidpad 12 ends with the steering frozen at
    // full lock), so an input-based test never terminates.
    size_t end = start;
    for (size_t i = start; i < rows.size(); ++i) {
        if (moving[i]) end = i;
    }
    return {start, std::max(start, end)};
}

// ============================================================================
// EVALUATE A SINGLE CANDIDATE INDIVIDUAL ACROSS ALL BAGS
// ============================================================================

// Reads one optional numeric field, falling back when the key is absent.
double yaml_number(const YAML::Node& node, const char* key, double fallback) {
    return node[key] ? node[key].as<double>() : fallback;
}

// Builds the per-dataset scoring configuration. Each dataset may override the
// tuning-wide score block; anything it does not set falls back to that block.
std::vector<DatasetConfig> read_dataset_configs(
    const YAML::Node& tuning_config, const YAML::Node& csvs,
    const std::vector<std::vector<CsvRow>>& all_csvs_rows) {
    const YAML::Node default_score_config = tuning_config["tuning"]["score"];

    std::vector<DatasetConfig> dataset_configs;
    for (size_t b = 0; b < csvs.size(); ++b) {
        const YAML::Node csv_node = csvs[b];
        DatasetConfig cfg;
        cfg.weight = yaml_number(csv_node, "weight", 1.0);
        cfg.start_offset_s = yaml_number(csv_node, "start_offset_s", 0.0);
        cfg.stop_after_s = yaml_number(csv_node, "stop_after_s", -1.0);
        cfg.initial_soc = yaml_number(csv_node, "initial_soc", -1.0);

        const YAML::Node score = csv_node["score"] ? csv_node["score"] : default_score_config;
        cfg.max_distance_error = yaml_number(score, "max_distance_error", 2.0);
        cfg.max_velocity_error = yaml_number(score, "max_velocity_error", 5.0);
        cfg.reanchor_period = yaml_number(score, "reanchor_period", 0.0);
        cfg.bias_yaw_rate_weight = yaml_number(score, "bias_yaw_rate_weight", 0.0);
        cfg.bias_velocity_weight = yaml_number(score, "bias_velocity_weight", 0.0);
        cfg.heading_weight = yaml_number(score, "heading_weight", 0.25);
        cfg.yaw_rate_weight = yaml_number(score, "yaw_rate_weight", 0.25);
        cfg.limit_weight = yaml_number(score, "limit_weight", 0.50);
        cfg.ignore_lateral_distance =
            score["ignore_lateral_distance"] && score["ignore_lateral_distance"].as<bool>();

        const std::pair<size_t, size_t> window = detect_motion_window(all_csvs_rows.at(b));
        cfg.start_index = window.first;
        cfg.end_index = window.second;
        std::cout << "Dataset " << b << " automatic motion window: "
                  << all_csvs_rows.at(b).at(window.first).timestamp_s << "s to "
                  << all_csvs_rows.at(b).at(window.second).timestamp_s << "s" << std::endl;
        dataset_configs.push_back(cfg);
    }
    return dataset_configs;
}

EvaluationResult evaluate_candidate(
    const std::vector<std::vector<CsvRow>>& all_csvs_rows,
    const std::vector<DatasetConfig>& dataset_configs,
    const std::vector<ParameterSpec>& param_specs,
    const std::vector<double>& candidate_values,
    const InvictaSimParameters& base_params
) {
    g_eval_count.fetch_add(1, std::memory_order_relaxed);
    InvictaSimParameters sim_params = base_params;
    sim_params.car_parameters = deep_copy_car_parameters(base_params.car_parameters);

    for (size_t i = 0; i < param_specs.size(); ++i) {
        apply_parameter_override(sim_params.car_parameters, param_specs[i].name, candidate_values[i]);
    }

    double weighted_score_sum = 0.0;
    double total_weight = 0.0;
    double weighted_position_rmse = 0.0;
    double weighted_heading_rmse = 0.0;
    double weighted_velocity_rmse = 0.0;
    double weighted_velocity_x_rmse = 0.0;
    double weighted_velocity_y_rmse = 0.0;
    double weighted_yaw_rate_rmse = 0.0;
    double weighted_front_wheel_rpm_rmse = 0.0;
    double weighted_motor_rpm_rmse = 0.0;
    double weighted_position_mean = 0.0;
    double weighted_velocity_mean = 0.0;
    double weighted_position_max = 0.0;
    double weighted_velocity_max = 0.0;
    double weighted_in_limit = 0.0;

    for (size_t b = 0; b < dataset_configs.size(); ++b) {
        const auto& ds_cfg = dataset_configs[b];
        double weight = ds_cfg.weight;
        double stop_duration = ds_cfg.stop_after_s;
        double max_dist_error = ds_cfg.max_distance_error;
        bool ignore_lateral = ds_cfg.ignore_lateral_distance;

        const auto& rows = all_csvs_rows[b];
        if (rows.empty()) {
            weighted_score_sum += weight * -1e9;
            total_weight += weight;
            continue;
        }

        const size_t start_idx = std::min(ds_cfg.start_index, rows.size() - 1);
        const size_t end_idx = std::min(ds_cfg.end_index, rows.size() - 1);

        double start_time = rows[start_idx].timestamp_s;

        // INSTANTIATE THE HARDCODED MODEL DIRECTLY
        if (!sim_params.car_parameters) {
            std::cerr << "Error: sim_params.car_parameters is null!" << std::endl;
        } else if (!sim_params.car_parameters->motor_parameters) {
            std::cerr << "Error: sim_params.car_parameters->motor_parameters is null!" << std::endl;
        }
        // Each recording starts at its own state of charge (measured from the
        // resting DC-bus voltage); the battery model must be seeded with it
        // before the model is built, since Thevenin latches it in its ctor.
        const double configured_soc = sim_params.car_parameters->battery_parameters->initial_soc;
        if (ds_cfg.initial_soc >= 0.0) {
            sim_params.car_parameters->battery_parameters->initial_soc = ds_cfg.initial_soc;
        }
        InlineFSFEUP02Model model(sim_params);
        sim_params.car_parameters->battery_parameters->initial_soc = configured_soc;

        // Warm only hidden actuator/energy states from the discarded prefix.
        // The scoring state is still set from the first accepted measurement;
        // this prevents the inverter delay from being spuriously restarted at
        // the automatically detected motion boundary.
        SimState warmup_state;
        for (size_t i = 1; i <= start_idx; ++i) {
            const double warmup_dt = rows[i].timestamp_s - rows[i - 1].timestamp_s;
            if (warmup_dt <= 0.0) continue;
            const CsvRow& command = rows[i - 1];
            model.step(warmup_dt,
                       common_lib::structures::Wheels(command.throttle_fl, command.throttle_fr,
                                                        command.throttle_rl, command.throttle_rr),
                       command.steering, warmup_state);
        }

        SimState state;
        CsvRow first_sample = rows[start_idx];
        state.x = 0.0;
        state.y = 0.0;
        state.yaw = 0.0;
        state.vx = first_sample.real_vx;
        state.vy = first_sample.real_vy;
        state.yaw_rate = first_sample.real_yaw_rate;
        
        initialise_wheel_speeds(state, first_sample, *sim_params.car_parameters);

        RunningRmse position_rmse, heading_rmse, velocity_rmse, velocity_x_rmse, velocity_x_slow_rmse;
        RunningRmse velocity_y_rmse, yaw_rate_rmse, yaw_rate_under_rmse, front_wheel_rpm_rmse, motor_rpm_rmse;

        PoseSample real_origin = {first_sample.real_x, first_sample.real_y, first_sample.real_yaw};
        PoseSample sim_origin = {state.x, state.y, state.yaw};

        double last_time = first_sample.timestamp_s;

        // Errors are measured over the *whole* window.  The previous behaviour --
        // stop accumulating at the first limit violation -- meant a candidate that
        // violated on its second sample was scored on one nearly-perfect sample and
        // so outranked every candidate that actually tracked the lap: the search was
        // being rewarded for failing early.  Instead every sample is scored, with a
        // cap so one blow-up cannot swamp the mean, and divergence is charged the cap.
        const double pos_error_cap = 10.0 * max_dist_error;
        const double vel_error_cap = 10.0 * ds_cfg.max_velocity_error;
        const double reanchor = ds_cfg.reanchor_period;
        double next_anchor = reanchor > 0.0 ? reanchor : 1e18;
        bool seg_diverged = false;  // physics blew up; charge the cap until re-anchor
        long in_limit_samples = 0, total_samples = 0;
        double signed_vx_error_sum = 0.0, signed_yaw_rate_error_sum = 0.0;
        for (size_t i = start_idx + 1; i <= end_idx; ++i) {
            const CsvRow& row = rows[i];
            if (stop_duration > 0.0 && (row.timestamp_s - start_time) > stop_duration) break;

            double dt = row.timestamp_s - last_time;
            if (dt <= 0.0) continue;
            last_time = row.timestamp_s;
            const double t_rel = row.timestamp_s - start_time;

            PoseSample real_pose = transform_pose_to_map(
                {row.real_x, row.real_y, row.real_yaw}, real_origin, sim_origin);

            // Re-anchor boundary: reset the sim to the measured state and open a
            // fresh open-loop horizon (vy is unmeasured, so seed 0).
            if (reanchor > 0.0 && t_rel >= next_anchor) {
                next_anchor += reanchor;
                seg_diverged = false;
                state = SimState();
                state.x = real_pose.x; state.y = real_pose.y; state.yaw = real_pose.yaw;
                state.vx = row.real_vx; state.vy = 0.0; state.yaw_rate = row.real_yaw_rate;
                initialise_wheel_speeds(state, row, *sim_params.car_parameters);
                continue;
            }

            double current_pos_error = pos_error_cap;
            double current_vel_error = vel_error_cap;
            double sim_front_rpm = 0.0, sim_motor_rpm = 0.0;
            double heading_error = M_PI, yaw_rate_error = 0.0;
            const double real_front_rpm = 0.5 * (row.real_fl_rpm + row.real_fr_rpm);

            if (!seg_diverged) {
                const CsvRow& prev_row = rows[i - 1];
                common_lib::structures::Wheels throttle(prev_row.throttle_fl, prev_row.throttle_fr,
                                                        prev_row.throttle_rl, prev_row.throttle_rr);

                // STEP THE PHYSICS ENGINE
                model.step(dt, throttle, prev_row.steering, state);

                if (!std::isfinite(state.x) || !std::isfinite(state.y) || !std::isfinite(state.vx) ||
                    !std::isfinite(state.vy) || !std::isfinite(state.yaw_rate) ||
                    std::abs(state.vx) > 80.0 || std::abs(state.vy) > 40.0 ||
                    std::abs(state.yaw_rate) > 20.0) {
                    seg_diverged = true;
                }
            }

            if (!seg_diverged) {
                sim_front_rpm = 0.5 * (state.wheels_speed.front_left + state.wheels_speed.front_right) *
                                60.0 / (2.0 * M_PI);
                sim_motor_rpm = state.motor_omega * 60.0 / (2.0 * M_PI);

                if (ignore_lateral) {
                    // Ignore lateral drift: just compare total distance traveled from start
                    double sim_dist = std::hypot(state.x - sim_origin.x, state.y - sim_origin.y);
                    double real_dist = std::hypot(real_pose.x - real_origin.x, real_pose.y - real_origin.y);
                    current_pos_error = std::abs(sim_dist - real_dist);
                } else {
                    // Strict 2D Euclidean distance error
                    current_pos_error = std::hypot(state.x - real_pose.x, state.y - real_pose.y);
                }
                // real_vy is not measured (identically 0), so velocity error is longitudinal.
                current_vel_error = std::abs(state.vx - row.real_vx);
                heading_error = normalize_angle(state.yaw - real_pose.yaw);
                yaw_rate_error = state.yaw_rate - row.real_yaw_rate;
            }

            ++total_samples;
            if (current_pos_error <= max_dist_error && current_vel_error <= ds_cfg.max_velocity_error)
                ++in_limit_samples;
            if (!seg_diverged) {
                signed_vx_error_sum += state.vx - row.real_vx;
                signed_yaw_rate_error_sum += yaw_rate_error;
            }

            position_rmse.update(std::min(current_pos_error, pos_error_cap));
            heading_rmse.update(heading_error);
            velocity_x_rmse.update(std::min(current_vel_error, vel_error_cap));
            velocity_x_slow_rmse.update(std::min(0.0, state.vx - row.real_vx));
            velocity_y_rmse.update(state.vy - row.real_vy);
            velocity_rmse.update(std::min(current_vel_error, vel_error_cap));
            yaw_rate_rmse.update(yaw_rate_error);
            yaw_rate_under_rmse.update(std::min(0.0, yaw_rate_error));
            front_wheel_rpm_rmse.update(sim_front_rpm - real_front_rpm);
            motor_rpm_rmse.update(sim_motor_rpm - row.real_motor_rpm);
        }

        const double in_limit_fraction =
            total_samples > 0 ? static_cast<double>(in_limit_samples) / total_samples : 0.0;

        // Score the two acceptance criteria directly, each normalised by its own
        // limit so neither unit dominates, plus a light pull on the dynamic states
        // that generate them, plus the two drift biases.  The bias references are
        // chosen so a typical bias contributes on the same order as the position
        // and velocity terms -- normalising by the level at which a bias stops
        // mattering over 135 s (0.002 rad/s) instead made the bias term 20x
        // everything else and the search optimised nothing but drift.
        constexpr double kYawRateBiasReference = 0.02;   // rad/s
        constexpr double kVelocityBiasReference = 0.30;  // m/s
        const double sample_count = std::max<double>(total_samples, 1);
        const double velocity_bias = signed_vx_error_sum / sample_count;
        const double yaw_rate_bias = signed_yaw_rate_error_sum / sample_count;
        const double dataset_score =
            -(position_rmse.mean() / std::max(max_dist_error, 1e-6) +
              velocity_rmse.mean() / std::max(ds_cfg.max_velocity_error, 1e-6) +
              ds_cfg.bias_yaw_rate_weight * std::abs(yaw_rate_bias) / kYawRateBiasReference +
              ds_cfg.bias_velocity_weight * std::abs(velocity_bias) / kVelocityBiasReference +
              ds_cfg.heading_weight * heading_rmse.get() +
              ds_cfg.yaw_rate_weight * yaw_rate_rmse.get() +
              ds_cfg.limit_weight * (1.0 - in_limit_fraction));

        weighted_score_sum += weight * dataset_score;
        weighted_position_rmse += weight * position_rmse.get();
        weighted_heading_rmse += weight * heading_rmse.get();
        weighted_velocity_rmse += weight * velocity_rmse.get();
        weighted_velocity_x_rmse += weight * velocity_x_rmse.get();
        weighted_velocity_y_rmse += weight * velocity_y_rmse.get();
        weighted_yaw_rate_rmse += weight * yaw_rate_rmse.get();
        weighted_front_wheel_rpm_rmse += weight * front_wheel_rpm_rmse.get();
        weighted_motor_rpm_rmse += weight * motor_rpm_rmse.get();
        weighted_position_mean += weight * position_rmse.mean();
        weighted_velocity_mean += weight * velocity_rmse.mean();
        weighted_position_max += weight * position_rmse.max();
        weighted_velocity_max += weight * velocity_rmse.max();
        weighted_in_limit += weight * in_limit_fraction;
        total_weight += weight;
    }

    const double inv_weight = 1.0 / std::max(total_weight, 1e-9);
    EvaluationResult res;
    res.total_score = weighted_score_sum * inv_weight;
    res.metrics.position_rmse = weighted_position_rmse * inv_weight;
    res.metrics.heading_rmse = weighted_heading_rmse * inv_weight;
    res.metrics.velocity_rmse = weighted_velocity_rmse * inv_weight;
    res.metrics.velocity_x_rmse = weighted_velocity_x_rmse * inv_weight;
    res.metrics.velocity_y_rmse = weighted_velocity_y_rmse * inv_weight;
    res.metrics.yaw_rate_rmse = weighted_yaw_rate_rmse * inv_weight;
    res.metrics.front_wheel_rpm_rmse = weighted_front_wheel_rpm_rmse * inv_weight;
    res.metrics.motor_rpm_rmse = weighted_motor_rpm_rmse * inv_weight;
    res.metrics.position_mean = weighted_position_mean * inv_weight;
    res.metrics.velocity_mean = weighted_velocity_mean * inv_weight;
    res.metrics.position_max = weighted_position_max * inv_weight;
    res.metrics.velocity_max = weighted_velocity_max * inv_weight;
    res.metrics.in_limit_fraction = weighted_in_limit * inv_weight;
    return res;
}

// Scores part of a population. Split out as a plain function so the parallel
// evaluation below needs no lambda, queue or future: the candidates are
// independent, so each thread simply owns a slice of the vector.
void evaluate_population_chunk(const std::vector<std::vector<CsvRow>>* all_csvs_rows,
                               const std::vector<DatasetConfig>* dataset_configs,
                               const std::vector<ParameterSpec>* param_specs,
                               const InvictaSimParameters* base_params,
                               std::vector<Individual>* population, size_t begin, size_t end) {
    for (size_t i = begin; i < end; ++i) {
        const EvaluationResult result = evaluate_candidate(
            *all_csvs_rows, *dataset_configs, *param_specs, (*population)[i].values, *base_params);
        (*population)[i].score = result.total_score;
        (*population)[i].metrics = result.metrics;
    }
}

void evaluate_population(const std::vector<std::vector<CsvRow>>& all_csvs_rows,
                         const std::vector<DatasetConfig>& dataset_configs,
                         const std::vector<ParameterSpec>& param_specs,
                         const InvictaSimParameters& base_params,
                         std::vector<Individual>& population) {
    const size_t count = population.size();
    if (count == 0) return;
    const size_t threads = std::min<size_t>(std::max(1u, std::thread::hardware_concurrency()), count);
    const size_t chunk = (count + threads - 1) / threads;

    std::vector<std::thread> workers;
    for (size_t begin = 0; begin < count; begin += chunk) {
        workers.push_back(std::thread(evaluate_population_chunk, &all_csvs_rows, &dataset_configs,
                                      &param_specs, &base_params, &population, begin,
                                      std::min(count, begin + chunk)));
    }
    for (size_t i = 0; i < workers.size(); ++i) {
        workers[i].join();
    }
}

// Picks the best of a few randomly drawn individuals.
const Individual& tournament_select(const std::vector<Individual>& population, int size,
                                    std::mt19937& rng) {
    size_t best = rng() % population.size();
    for (int t = 1; t < size; ++t) {
        const size_t index = rng() % population.size();
        if (population[index].score > population[best].score) best = index;
    }
    return population[best];
}

Individual run_genetic_algorithm(
    const std::vector<std::vector<CsvRow>>& all_csvs_rows,
    const std::vector<ParameterSpec>& param_specs,
    const YAML::Node& tuning_config,
    const InvictaSimParameters& base_params
) {
    YAML::Node opt = tuning_config["tuning"]["optimizer"];
    int generations = opt["generations"] ? opt["generations"].as<int>() : 15;
    int population_size = opt["population_size"] ? opt["population_size"].as<int>() : 24;
    int elite_count = opt["elite_count"] ? opt["elite_count"].as<int>() : 4;
    int immigrant_count = opt["immigrant_count"] ? opt["immigrant_count"].as<int>() : 3;
    double mutation_rate = opt["mutation_rate"] ? opt["mutation_rate"].as<double>() : 0.35;
    double mutation_scale = opt["mutation_scale"] ? opt["mutation_scale"].as<double>() : 0.16;
    double blend_alpha = opt["blend_alpha"] ? opt["blend_alpha"].as<double>() : 0.25;
    int tournament_size = opt["tournament_size"] ? opt["tournament_size"].as<int>() : 3;
    int seed = opt["seed"] ? opt["seed"].as<int>() : 42;

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist_01(0.0, 1.0);
    
    // Create a thread pool with the maximum hardware threads available
    std::vector<double> baseline_vals(param_specs.size());
    for (size_t p = 0; p < param_specs.size(); ++p) {
        baseline_vals[p] = std::clamp(get_baseline_value(base_params.car_parameters, param_specs[p].name), param_specs[p].min_val, param_specs[p].max_val);
    }

    std::vector<Individual> population(population_size);
    population[0].values = baseline_vals;

    for (int i = 1; i < population_size; ++i) {
        population[i].values.resize(param_specs.size());
        for (size_t p = 0; p < param_specs.size(); ++p) {
            std::uniform_real_distribution<double> dist(param_specs[p].min_val, param_specs[p].max_val);
            population[i].values[p] = dist(rng);
        }
    }

    Individual global_best;
    global_best.score = -1e9;
    global_best.values = population[0].values;

    // Read once, not per generation: parsing the YAML and re-detecting the motion
    // window every generation only repeated the same work and the same log lines.
    YAML::Node csvs = tuning_config["tuning"]["csvs"];
    if (!csvs && tuning_config["tuning"]["bags"]) {
        csvs = tuning_config["tuning"]["bags"];
    }
    const std::vector<DatasetConfig> dataset_configs =
        read_dataset_configs(tuning_config, csvs, all_csvs_rows);

    for (int gen = 0; gen < generations; ++gen) {
        if (should_stop()) break;
        std::cout << "--- Generation " << gen + 1 << "/" << generations << " ---" << std::endl;

        evaluate_population(all_csvs_rows, dataset_configs, param_specs, base_params, population);

        for (int i = 0; i < population_size; ++i) {
            if (population[i].score > global_best.score) {
                global_best = population[i];
                log_convergence("ga", global_best.score, global_best.metrics.position_rmse, global_best.metrics.velocity_rmse);
                std::cout << "  [New Global Best] Score: " << global_best.score << " (Pos RMSE: " << global_best.metrics.position_rmse << "m) Parameters:";
                for (size_t p = 0; p < param_specs.size(); ++p) {
                    std::cout << " " << param_specs[p].name << "=" << global_best.values[p];
                }
                std::cout << std::endl;
            }
        }

        std::sort(population.begin(), population.end(), [](const Individual& a, const Individual& b) {
            return a.score > b.score;
        });

        std::cout << "  Best score in generation: " << population[0].score << std::endl;

        // Iteratively save best parameters
        std::string target_out_path = "src/invictasim/tuning_csvs/best_parameters.yaml";
        if (std::filesystem::exists("/home/ws/src/invictasim/tuning_csvs")) {
            target_out_path = "/home/ws/src/invictasim/tuning_csvs/best_parameters.yaml";
        } else {
            std::filesystem::create_directories("src/invictasim/tuning_csvs");
        }
        std::ofstream best_file(target_out_path);
        if (best_file.is_open()) {
            best_file << "generation: " << gen + 1 << "\n";
            best_file << "best_score: " << global_best.score << "\n";
            best_file << "metrics:\n";
            best_file << "  position_mean: " << global_best.metrics.position_mean << "\n";
            best_file << "  velocity_mean: " << global_best.metrics.velocity_mean << "\n";
            best_file << "  position_max: " << global_best.metrics.position_max << "\n";
            best_file << "  velocity_max: " << global_best.metrics.velocity_max << "\n";
            best_file << "  in_limit_fraction: " << global_best.metrics.in_limit_fraction << "\n";
            best_file << "  position_rmse: " << global_best.metrics.position_rmse << "\n";
            best_file << "  heading_rmse: " << global_best.metrics.heading_rmse << "\n";
            best_file << "  velocity_rmse: " << global_best.metrics.velocity_rmse << "\n";
            best_file << "  velocity_x_rmse: " << global_best.metrics.velocity_x_rmse << "\n";
            best_file << "  velocity_y_rmse: " << global_best.metrics.velocity_y_rmse << "\n";
            best_file << "  yaw_rate_rmse: " << global_best.metrics.yaw_rate_rmse << "\n";
            best_file << "  front_wheel_rpm_rmse: " << global_best.metrics.front_wheel_rpm_rmse << "\n";
            best_file << "  motor_rpm_rmse: " << global_best.metrics.motor_rpm_rmse << "\n";
            best_file << "parameters:\n";
            for (size_t p = 0; p < param_specs.size(); ++p) {
                best_file << "  " << param_specs[p].name << ": " << global_best.values[p] << "\n";
            }
            best_file.close();
        }

        // if (global_best.position_rmse <= 0.1) {
        //     std::cout << "Optimization stopped: Target Position RMSE (<= 0.1m) reached!" << std::endl;
        //     break;
        // }

        std::vector<Individual> next_pop;
        next_pop.reserve(population_size);

        for (int i = 0; i < std::min(elite_count, population_size); ++i) {
            next_pop.push_back(population[i]);
        }

        for (int i = 0; i < immigrant_count; ++i) {
            Individual immigrant;
            immigrant.values.resize(param_specs.size());
            for (size_t p = 0; p < param_specs.size(); ++p) {
                std::uniform_real_distribution<double> dist(param_specs[p].min_val, param_specs[p].max_val);
                immigrant.values[p] = dist(rng);
            }
            next_pop.push_back(immigrant);
        }

        while (next_pop.size() < static_cast<size_t>(population_size)) {
            const Individual& parent_a = tournament_select(population, tournament_size, rng);
            const Individual& parent_b = tournament_select(population, tournament_size, rng);

            Individual child;
            child.values.resize(param_specs.size());
            for (size_t p = 0; p < param_specs.size(); ++p) {
                std::uniform_real_distribution<double> dist_blend(-blend_alpha, 1.0 + blend_alpha);
                double alpha = dist_blend(rng);
                double val = alpha * parent_a.values[p] + (1.0 - alpha) * parent_b.values[p];
                child.values[p] = std::clamp(val, param_specs[p].min_val, param_specs[p].max_val);
            }

            for (size_t p = 0; p < param_specs.size(); ++p) {
                if (dist_01(rng) < mutation_rate) {
                    double span = param_specs[p].max_val - param_specs[p].min_val;
                    std::normal_distribution<double> dist_normal(0.0, mutation_scale * span);
                    child.values[p] = std::clamp(child.values[p] + dist_normal(rng), param_specs[p].min_val, param_specs[p].max_val);
                }
            }
            next_pop.push_back(child);
        }
        population = next_pop;
    }
    return global_best;
}

// Everything one annealing chain needs. Grouped into a struct so the chain can be
// an ordinary function rather than a lambda capturing a dozen locals by
// reference, which also makes what is shared between threads explicit: only the
// last three members are, and they are guarded by best_mutex.
struct AnnealingSearch {
    const std::vector<std::vector<CsvRow>>* all_csvs_rows;
    const std::vector<DatasetConfig>* dataset_configs;
    const std::vector<ParameterSpec>* param_specs;
    const InvictaSimParameters* base_params;
    const std::vector<double>* start_values;

    int iterations;
    int total_iterations;
    double initial_temp;
    double cooling_rate;
    double mutation_scale;

    std::mutex* best_mutex;
    Individual* global_best;
    std::atomic<int>* completed_iterations;
};

// Writes the running best to disk so a long search can be inspected or applied
// without waiting for it to finish.
void write_best_parameters(const Individual& best, const std::vector<ParameterSpec>& param_specs,
                           int iteration, int chain_id) {
    std::string path = "src/invictasim/tuning_csvs/best_parameters.yaml";
    if (std::filesystem::exists("/home/ws/src/invictasim/tuning_csvs")) {
        path = "/home/ws/src/invictasim/tuning_csvs/best_parameters.yaml";
    } else {
        std::error_code ec;
        std::filesystem::create_directories("src/invictasim/tuning_csvs", ec);
    }

    std::ofstream file(path);
    if (!file.is_open()) return;

    file << "iteration: " << iteration << "\n";
    file << "chain: " << chain_id << "\n";
    file << "best_score: " << best.score << "\n";
    file << "metrics:\n";
    file << "  position_mean: " << best.metrics.position_mean << "\n";
    file << "  velocity_mean: " << best.metrics.velocity_mean << "\n";
    file << "  position_max: " << best.metrics.position_max << "\n";
    file << "  velocity_max: " << best.metrics.velocity_max << "\n";
    file << "  in_limit_fraction: " << best.metrics.in_limit_fraction << "\n";
    file << "  position_rmse: " << best.metrics.position_rmse << "\n";
    file << "  heading_rmse: " << best.metrics.heading_rmse << "\n";
    file << "  velocity_rmse: " << best.metrics.velocity_rmse << "\n";
    file << "  velocity_x_rmse: " << best.metrics.velocity_x_rmse << "\n";
    file << "  velocity_y_rmse: " << best.metrics.velocity_y_rmse << "\n";
    file << "  yaw_rate_rmse: " << best.metrics.yaw_rate_rmse << "\n";
    file << "  front_wheel_rpm_rmse: " << best.metrics.front_wheel_rpm_rmse << "\n";
    file << "  motor_rpm_rmse: " << best.metrics.motor_rpm_rmse << "\n";
    file << "parameters:\n";
    for (size_t p = 0; p < param_specs.size(); ++p) {
        file << "  " << param_specs[p].name << ": " << best.values[p] << "\n";
    }
}

// Perturbs a small random subset of the coordinates. Full-vector Gaussian moves
// are almost always rejected in this many dimensions; mutating 1-3 coordinates
// gives a far higher acceptance rate on this expensive black box.
void mutate_candidate(std::vector<double>& values, const std::vector<ParameterSpec>& param_specs,
                      double mutation_scale, std::mt19937& rng) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_int_distribution<size_t> coordinate(0, param_specs.size() - 1);
    const int count = 1 + static_cast<int>(unit(rng) * 3.0);
    for (int m = 0; m < count; ++m) {
        const size_t p = coordinate(rng);
        const double span = param_specs[p].max_val - param_specs[p].min_val;
        std::normal_distribution<double> step(0.0, mutation_scale * span);
        values[p] = std::clamp(values[p] + step(rng), param_specs[p].min_val, param_specs[p].max_val);
    }
}

void run_annealing_chain(AnnealingSearch search, int chain_id, int chain_seed) {
    const std::vector<ParameterSpec>& param_specs = *search.param_specs;
    std::mt19937 rng(chain_seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    Individual current;
    current.values = *search.start_values;
    // Chains after the first start slightly displaced so they explore different
    // neighbourhoods instead of retracing one path.
    if (chain_id > 0) {
        for (size_t p = 0; p < param_specs.size(); ++p) {
            const double span = param_specs[p].max_val - param_specs[p].min_val;
            std::normal_distribution<double> offset(0.0, 0.05 * span);
            current.values[p] = std::clamp(current.values[p] + offset(rng), param_specs[p].min_val,
                                           param_specs[p].max_val);
        }
    }

    EvaluationResult result = evaluate_candidate(*search.all_csvs_rows, *search.dataset_configs,
                                                 param_specs, current.values, *search.base_params);
    current.score = result.total_score;
    current.metrics = result.metrics;

    Individual local_best = current;
    double temp = search.initial_temp;

    for (int iter = 0; iter < search.iterations; ++iter) {
        if (should_stop()) break;

        Individual candidate;
        candidate.values = current.values;
        mutate_candidate(candidate.values, param_specs, search.mutation_scale, rng);

        result = evaluate_candidate(*search.all_csvs_rows, *search.dataset_configs, param_specs,
                                    candidate.values, *search.base_params);
        candidate.score = result.total_score;
        candidate.metrics = result.metrics;

        const double delta = candidate.score - current.score;
        if (delta > 0.0 || std::exp(delta / temp) > unit(rng)) {
            current = candidate;
        }

        if (current.score > local_best.score) {
            local_best = current;
            std::lock_guard<std::mutex> lock(*search.best_mutex);
            if (local_best.score > search.global_best->score) {
                *search.global_best = local_best;
                log_convergence("sa", search.global_best->score,
                                search.global_best->metrics.position_rmse,
                                search.global_best->metrics.velocity_rmse);
                std::cout << "\n[Chain " << chain_id << "] New Global Best! Score: "
                          << search.global_best->score
                          << " | Pos RMSE: " << search.global_best->metrics.position_rmse
                          << "m\nParameters:";
                for (size_t p = 0; p < param_specs.size(); ++p) {
                    std::cout << " " << param_specs[p].name << "=" << search.global_best->values[p];
                }
                std::cout << std::endl;
                write_best_parameters(*search.global_best, param_specs, iter + 1, chain_id);
            }
        }

        temp *= search.cooling_rate;

        const int done = ++(*search.completed_iterations);
        if (done % 1000 == 0) {
            std::cout << "Progress: " << done << " / " << search.total_iterations
                      << " evaluations...\r" << std::flush;
        }
    }
}

Individual run_simulated_annealing(
    const std::vector<std::vector<CsvRow>>& all_csvs_rows,
    const std::vector<ParameterSpec>& param_specs,
    const YAML::Node& tuning_config,
    const InvictaSimParameters& base_params
) {
    const YAML::Node opt = tuning_config["tuning"]["optimizer"];
    const int iterations = static_cast<int>(yaml_number(opt, "iterations", 10000));
    const int seed = static_cast<int>(yaml_number(opt, "seed", 42));
    const int num_chains = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));

    YAML::Node csvs = tuning_config["tuning"]["csvs"];
    if (!csvs && tuning_config["tuning"]["bags"]) {
        csvs = tuning_config["tuning"]["bags"];
    }
    const std::vector<DatasetConfig> dataset_configs =
        read_dataset_configs(tuning_config, csvs, all_csvs_rows);

    std::vector<double> start_values(param_specs.size());
    for (size_t p = 0; p < param_specs.size(); ++p) {
        start_values[p] =
            std::clamp(get_baseline_value(base_params.car_parameters, param_specs[p].name),
                       param_specs[p].min_val, param_specs[p].max_val);
    }

    std::mutex best_mutex;
    Individual global_best;
    global_best.score = -1e9;
    std::atomic<int> completed_iterations(0);

    AnnealingSearch search;
    search.all_csvs_rows = &all_csvs_rows;
    search.dataset_configs = &dataset_configs;
    search.param_specs = &param_specs;
    search.base_params = &base_params;
    search.start_values = &start_values;
    search.iterations = iterations;
    search.total_iterations = iterations * num_chains;
    search.initial_temp = yaml_number(opt, "initial_temp", 10.0);
    search.cooling_rate = yaml_number(opt, "cooling_rate", 0.999);
    search.mutation_scale = yaml_number(opt, "mutation_scale", 0.16);
    search.best_mutex = &best_mutex;
    search.global_best = &global_best;
    search.completed_iterations = &completed_iterations;

    std::cout << "Starting Parallel Simulated Annealing with " << num_chains
              << " independent chains (Total iterations: " << search.total_iterations << ")..."
              << std::endl;

    std::vector<std::thread> threads;
    for (int i = 0; i < num_chains; ++i) {
        threads.push_back(std::thread(run_annealing_chain, search, i, seed + i));
    }
    for (size_t i = 0; i < threads.size(); ++i) {
        threads[i].join();
    }
    std::cout << std::endl;

    return global_best;
}

// ============================================================================
// DIAGNOSTIC: replay one dataset with the baseline (or overridden) parameters
// and dump per-step sim-vs-real telemetry.  Never breaks on error so the full
// divergence profile is visible.  Reports the first time each survival limit is
// exceeded.  This is the tool used to localise physics bugs.
// ============================================================================
void run_diagnostic(
    const std::vector<std::vector<CsvRow>>& all_csvs_rows,
    const std::vector<ParameterSpec>& param_specs,
    const std::vector<double>& override_values,
    const InvictaSimParameters& base_params,
    size_t dataset_index,
    double max_dist_error,
    double max_velocity_error,
    const std::string& out_path,
    double reanchor_period = 0.0,
    double initial_soc = -1.0
) {
    InvictaSimParameters sim_params = base_params;
    sim_params.car_parameters = deep_copy_car_parameters(base_params.car_parameters);
    for (size_t i = 0; i < param_specs.size() && i < override_values.size(); ++i) {
        apply_parameter_override(sim_params.car_parameters, param_specs[i].name, override_values[i]);
    }
    // Seed the pack at the SOC this recording actually started from.
    if (initial_soc >= 0.0) sim_params.car_parameters->battery_parameters->initial_soc = initial_soc;
    std::error_code dir_ec;
    std::filesystem::create_directories(std::filesystem::path(out_path).parent_path(), dir_ec);

    const auto& rows = all_csvs_rows.at(dataset_index);
    const auto window = detect_motion_window(rows);
    const size_t start_idx = window.first;
    const size_t end_idx = window.second;
    const double start_time = rows[start_idx].timestamp_s;

    InlineFSFEUP02Model model(sim_params);
    model.update_cache();

    // Warm hidden actuator/energy states over the discarded prefix.
    SimState warmup_state;
    for (size_t i = 1; i <= start_idx; ++i) {
        const double warmup_dt = rows[i].timestamp_s - rows[i - 1].timestamp_s;
        if (warmup_dt <= 0.0) continue;
        const CsvRow& c = rows[i - 1];
        model.step(warmup_dt,
                   common_lib::structures::Wheels(c.throttle_fl, c.throttle_fr, c.throttle_rl, c.throttle_rr),
                   c.steering, warmup_state);
    }

    SimState state;
    CsvRow first_sample = rows[start_idx];
    state.x = 0.0; state.y = 0.0; state.yaw = 0.0;
    state.vx = first_sample.real_vx;
    state.vy = first_sample.real_vy;
    state.yaw_rate = first_sample.real_yaw_rate;
    initialise_wheel_speeds(state, first_sample, *sim_params.car_parameters);

    PoseSample real_origin = {first_sample.real_x, first_sample.real_y, first_sample.real_yaw};
    PoseSample sim_origin = {state.x, state.y, state.yaw};
    double last_time = first_sample.timestamp_s;

    std::ofstream out(out_path);
    out << "t_rel,sim_x,sim_y,real_x,real_y,pos_err,sim_vx,sim_vy,real_vx,real_vy,vel_err,"
        << "sim_yaw,real_yaw,yaw_err,sim_yaw_rate,real_yaw_rate,sim_motor_rpm,real_motor_rpm,"
        << "sim_front_rpm,real_front_rpm,throttle_rl,steering\n";

    double first_pos_violation = -1.0, first_vel_violation = -1.0;
    double max_pos_err = 0.0, max_vel_err = 0.0;
    // Re-anchored analysis: reset the sim to the measured state every
    // reanchor_period seconds to expose the model's *local* fidelity and
    // systematic biases across the whole run (not just the first divergence).
    double next_anchor = reanchor_period > 0.0 ? reanchor_period : 1e18;
    int seg_total = 0, seg_survived = 0;
    double seg_max_pos = 0.0, seg_max_vel = 0.0;
    long bias_n = 0;
    double bias_vx = 0.0, bias_yawrate = 0.0, abs_yawrate = 0.0;
    // The acceptance criterion is a mean error over the whole recording, so the
    // means are accumulated on a uniform time grid rather than per telemetry row.
    double sum_pos = 0.0, sum_vel = 0.0, sum_pos_sq = 0.0, sum_vel_sq = 0.0;
    long in_limit_n = 0;
    for (size_t i = start_idx + 1; i <= end_idx; ++i) {
        const CsvRow& row = rows[i];
        double dt = row.timestamp_s - last_time;
        if (dt <= 0.0) continue;
        last_time = row.timestamp_s;
        const CsvRow& prev = rows[i - 1];
        model.step(dt, common_lib::structures::Wheels(prev.throttle_fl, prev.throttle_fr, prev.throttle_rl, prev.throttle_rr), prev.steering, state);

        PoseSample real_pose = transform_pose_to_map({row.real_x, row.real_y, row.real_yaw}, real_origin, sim_origin);
        double pos_err = std::hypot(state.x - real_pose.x, state.y - real_pose.y);
        double vel_err = std::abs(state.vx - row.real_vx)  /* real_vy is not measured (identically 0), so velocity error is longitudinal */;
        double t_rel = row.timestamp_s - start_time;
        if (first_pos_violation < 0.0 && pos_err > max_dist_error) first_pos_violation = t_rel;
        if (first_vel_violation < 0.0 && vel_err > max_velocity_error) first_vel_violation = t_rel;
        max_pos_err = std::max(max_pos_err, pos_err);
        max_vel_err = std::max(max_vel_err, vel_err);

        // Per-segment bias accounting (uses the current, possibly re-anchored state).
        bias_vx += state.vx - row.real_vx;
        bias_yawrate += state.yaw_rate - row.real_yaw_rate;
        abs_yawrate += std::abs(row.real_yaw_rate);
        ++bias_n;
        sum_pos += pos_err; sum_pos_sq += pos_err * pos_err;
        sum_vel += vel_err; sum_vel_sq += vel_err * vel_err;
        if (pos_err <= max_dist_error && vel_err <= max_velocity_error) ++in_limit_n;
        seg_max_pos = std::max(seg_max_pos, pos_err);
        seg_max_vel = std::max(seg_max_vel, vel_err);
        if (reanchor_period > 0.0 && t_rel >= next_anchor) {
            ++seg_total;
            if (seg_max_pos <= max_dist_error && seg_max_vel <= max_velocity_error) ++seg_survived;
            seg_max_pos = 0.0; seg_max_vel = 0.0;
            next_anchor += reanchor_period;
            // Reset sim to measured state; vy unmeasured so seed 0.
            state.x = real_pose.x; state.y = real_pose.y; state.yaw = real_pose.yaw;
            state.vx = row.real_vx; state.vy = 0.0; state.yaw_rate = row.real_yaw_rate;
            state.ax = 0.0; state.ay = 0.0;
            initialise_wheel_speeds(state, row, *sim_params.car_parameters);
        }

        double sim_motor_rpm = state.motor_omega * 60.0 / (2.0 * M_PI);
        double sim_front_rpm = 0.5 * (state.wheels_speed.front_left + state.wheels_speed.front_right) * 60.0 / (2.0 * M_PI);
        double real_front_rpm = 0.5 * (row.real_fl_rpm + row.real_fr_rpm);
        out << t_rel << ',' << state.x << ',' << state.y << ',' << real_pose.x << ',' << real_pose.y << ','
            << pos_err << ',' << state.vx << ',' << state.vy << ',' << row.real_vx << ',' << row.real_vy << ','
            << vel_err << ',' << state.yaw << ',' << real_pose.yaw << ',' << normalize_angle(state.yaw - real_pose.yaw) << ','
            << state.yaw_rate << ',' << row.real_yaw_rate << ',' << sim_motor_rpm << ',' << row.real_motor_rpm << ','
            << sim_front_rpm << ',' << real_front_rpm << ',' << prev.throttle_rl << ',' << prev.steering << '\n';

        if (!std::isfinite(state.vx)) { std::cout << "NON-FINITE at t=" << t_rel << "\n"; break; }
    }
    out.close();
    std::cout << "Diagnostic dataset " << dataset_index << " window " << start_time << "s.." << rows[end_idx].timestamp_s
              << "s (" << (rows[end_idx].timestamp_s - start_time) << "s driving)\n";
    std::cout << "  first pos-limit (" << max_dist_error << "m) violation: "
              << (first_pos_violation < 0 ? std::string("never") : std::to_string(first_pos_violation) + "s")
              << "  | max pos err " << max_pos_err << "m\n";
    std::cout << "  first vel-limit (" << max_velocity_error << "m/s) violation: "
              << (first_vel_violation < 0 ? std::string("never") : std::to_string(first_vel_violation) + "s")
              << "  | max vel err " << max_vel_err << "m/s\n";
    if (bias_n > 0) {
        std::cout << "  ACCEPTANCE: mean|pos err| " << (sum_pos / bias_n) << " m (limit " << max_dist_error
                  << ")  |  mean|vel err| " << (sum_vel / bias_n) << " m/s (limit " << max_velocity_error << ")\n";
        std::cout << "              pos RMSE " << std::sqrt(sum_pos_sq / bias_n) << " m, vel RMSE "
                  << std::sqrt(sum_vel_sq / bias_n) << " m/s, "
                  << (100.0 * in_limit_n / bias_n) << "% of samples inside both limits\n";
        std::cout << "  mean signed bias  vx=" << (bias_vx / bias_n) << " m/s  yaw_rate="
                  << (bias_yawrate / bias_n) << " rad/s (mean |real yaw_rate|=" << (abs_yawrate / bias_n) << ")\n";
    }
    if (reanchor_period > 0.0 && seg_total > 0) {
        std::cout << "  re-anchored every " << reanchor_period << "s: " << seg_survived << "/" << seg_total
                  << " segments stay within limits (" << (100.0 * seg_survived / seg_total) << "%)\n";
    }
    std::cout << "  wrote " << out_path << "\n";
}

std::string get_full_csv_path(const std::string& data_dir, const std::string& rel_path) {
    if (std::filesystem::path(rel_path).is_absolute() || data_dir.empty()) {
        return rel_path;
    }
    return (std::filesystem::path(data_dir) / rel_path).string();
}

int main(int argc, char** argv) {
    std::string config_path;
    std::string data_dir;
    bool diagnose = false;
    double g_reanchor_period = 0.0;
    std::vector<std::pair<std::string, double>> cli_overrides;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--data-dir" && i + 1 < argc) {
            data_dir = argv[++i];
        } else if (arg == "--diagnose") {
            diagnose = true;
        } else if (arg == "--reanchor" && i + 1 < argc) {
            g_reanchor_period = std::stod(argv[++i]);
        } else if (arg == "--set" && i + 1 < argc) {
            std::string kv = argv[++i];
            auto eq = kv.find('=');
            if (eq != std::string::npos)
                cli_overrides.emplace_back(kv.substr(0, eq), std::stod(kv.substr(eq + 1)));
        } else if (arg == "--params" && i + 1 < argc) {
            YAML::Node pf = YAML::LoadFile(argv[++i]);
            YAML::Node pnode = pf["parameters"] ? pf["parameters"] : pf;
            for (auto it = pnode.begin(); it != pnode.end(); ++it)
                cli_overrides.emplace_back(it->first.as<std::string>(), it->second.as<double>());
        } else if (arg == "--time-limit" && i + 1 < argc) {
            double secs = std::stod(argv[++i]);
            g_deadline = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(secs));
            g_has_deadline.store(true);
        }
    }

    g_start_time = std::chrono::steady_clock::now();
    std::signal(SIGINT, handle_sigint);
    if (!diagnose) {
        std::error_code ec;
        std::filesystem::create_directories("/home/ws/performance/invictasim_tuning", ec);
        std::ofstream log_init(g_convergence_log_path, std::ios::trunc);
        if (log_init.is_open()) log_init << "elapsed_s,evals,tag,score,position_rmse,velocity_rmse\n";
    }

    if (config_path.empty() || data_dir.empty()) {
        std::cerr << "Usage: " << argv[0] << " --config <config_yaml> --data-dir <data_csv_directory>" << std::endl;
        return 1;
    }

    std::cout << "Loading tuning configuration from: " << config_path << std::endl;
    YAML::Node tuning_config = YAML::LoadFile(config_path);

    // Initializing rclcpp is optional here unless your parameter config loader strictly requires it!
    // rclcpp::init(0, nullptr); 

    InvictaSimParameters base_params;
    
    // Using default base_params because it loads the full vehicle_models properly.

    YAML::Node csvs = tuning_config["tuning"]["csvs"];
    if (!csvs && tuning_config["tuning"]["bags"]) {
        csvs = tuning_config["tuning"]["bags"];
    }
    std::vector<std::vector<CsvRow>> all_csvs_rows;
    all_csvs_rows.reserve(csvs.size());

    const YAML::Node score_node = tuning_config["tuning"]["score"];
    const double resample_hz =
        (score_node && score_node["resample_hz"]) ? score_node["resample_hz"].as<double>() : 0.0;


    std::cout << "Loading CSV telemetry datasets..." << std::endl;
    for (size_t b = 0; b < csvs.size(); ++b) {
        std::string full_path;
        if (csvs[b]["csv_path"]) {
            full_path = get_full_csv_path(data_dir, csvs[b]["csv_path"].as<std::string>());
        } else if (csvs[b]["name"]) {
            full_path = (std::filesystem::path(data_dir) / (csvs[b]["name"].as<std::string>() + ".csv")).string();
        } else if (csvs[b]["path"]) {
            std::string rel_path = csvs[b]["path"].as<std::string>();
            full_path = get_full_csv_path(data_dir, rel_path);
        } else {
            std::cerr << "Warning: Bag entry is missing path or name." << std::endl;
            continue;
        }
        std::cout << "  Reading: " << full_path << std::endl;
        auto rows = read_csv(full_path);
        condition_reference(rows);
        const size_t raw_samples = rows.size();
        if (resample_hz > 0.0) rows = resample_rows(rows, resample_hz);
        std::cout << "    Loaded " << raw_samples << " samples (reference conditioned)";
        if (resample_hz > 0.0) std::cout << " -> " << rows.size() << " at " << resample_hz << " Hz";
        std::cout << "." << std::endl;
        all_csvs_rows.push_back(std::move(rows));
    }

    std::vector<ParameterSpec> param_specs;
    YAML::Node params_node = tuning_config["tuning"]["parameters"];
    if (params_node) {
        for (size_t i = 0; i < params_node.size(); ++i) {
            ParameterSpec spec;
            spec.name = params_node[i]["name"].as<std::string>();
            spec.min_val = params_node[i]["min"].as<double>();
            spec.max_val = params_node[i]["max"].as<double>();
            param_specs.push_back(spec);
        }
    }

    std::cout << "Loaded " << param_specs.size() << " optimization parameters." << std::endl;

    // Applied to both paths: in --diagnose they select the point to evaluate, and
    // in a search they move the starting point, which is how a run is warm-started
    // from a known-good incumbent instead of restarting cold in 30+ dimensions.
    for (const auto& ov : cli_overrides) {
        std::cout << "  override " << ov.first << " = " << ov.second << "\n";
        apply_parameter_override(base_params.car_parameters, ov.first, ov.second);
    }

    if (diagnose) {
        std::vector<double> baseline_vals(param_specs.size());
        for (size_t p = 0; p < param_specs.size(); ++p)
            baseline_vals[p] = std::clamp(get_baseline_value(base_params.car_parameters, param_specs[p].name), param_specs[p].min_val, param_specs[p].max_val);
        YAML::Node score_cfg = tuning_config["tuning"]["score"];
        double max_d = score_cfg["max_distance_error"] ? score_cfg["max_distance_error"].as<double>() : 1.5;
        double max_v = score_cfg["max_velocity_error"] ? score_cfg["max_velocity_error"].as<double>() : 2.0;
        for (size_t b = 0; b < all_csvs_rows.size(); ++b) {
            std::string out_path = "/home/ws/performance/invictasim_tuning/diagnostic_" + std::to_string(b) + ".csv";
            const double soc = (b < csvs.size() && csvs[b]["initial_soc"])
                                   ? csvs[b]["initial_soc"].as<double>() : -1.0;
            run_diagnostic(all_csvs_rows, param_specs, baseline_vals, base_params, b, max_d, max_v, out_path, g_reanchor_period, soc);
        }
        return 0;
    }

    // Extract the vehicle model from the tuning YAML
    std::string requested_model = "fsfeup02"; // default
    if (tuning_config["tuning"]["vehicle_model"]) {
        requested_model = tuning_config["tuning"]["vehicle_model"].as<std::string>();
    }
    std::cout << "Selected vehicle model from configuration: " << requested_model << std::endl;

    std::string algorithm = "genetic_algorithm";
    if (tuning_config["tuning"]["optimizer"]["algorithm"]) {
        algorithm = tuning_config["tuning"]["optimizer"]["algorithm"].as<std::string>();
    }
    std::cout << "Starting " << algorithm << " search..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    
    Individual optimal;

    // Only the 02 model has an inline counterpart; add a branch here if another
    // one is written.
    if (requested_model == "fsfeup02" || requested_model == "inline_02") {
        if (algorithm == "simulated_annealing") {
            optimal = run_simulated_annealing(all_csvs_rows, param_specs, tuning_config, base_params);
        } else {
            optimal = run_genetic_algorithm(all_csvs_rows, param_specs, tuning_config, base_params);
        }
    } else {
        std::cerr << "Error: Unknown or unlinked vehicle model '" << requested_model << "'." << std::endl;
        return 1;
    }

    auto end_time = std::chrono::steady_clock::now();
    double elapsed_s = std::chrono::duration<double>(end_time - start_time).count();
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "Optimization Complete in " << elapsed_s << " seconds." << std::endl;
    std::cout << "Optimal Cost Score: " << optimal.score << std::endl;
    std::cout << "Optimal Parameters:" << std::endl;
    for (size_t p = 0; p < param_specs.size(); ++p) {
        std::cout << "  " << param_specs[p].name << ": " << optimal.values[p] << std::endl;
    }
    std::cout << "========================================" << std::endl;

    // rclcpp::shutdown();
    return 0;
}
