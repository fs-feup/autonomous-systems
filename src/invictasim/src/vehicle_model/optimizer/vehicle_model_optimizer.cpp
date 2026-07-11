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

struct CsvRow {
    double timestamp_s, throttle_fl, throttle_fr, throttle_rl, throttle_rr, steering;
    double real_x, real_y, real_yaw, real_vx, real_vy, real_yaw_rate;
    double real_fl_rpm, real_fr_rpm, real_rl_rpm, real_rr_rpm, real_motor_rpm;
};

struct RunningRmse {
    double sum_squares = 0.0;
    int count = 0;
    void update(double value) { sum_squares += value * value; count++; }
    double get() const { return count <= 0 ? 0.0 : std::sqrt(sum_squares / static_cast<double>(count)); }
};

struct ParameterSpec { std::string name; double min_val; double max_val; };
struct Individual { std::vector<double> values; double score = 1e9; };
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

    auto get_col_idx = [&](const std::string& name) -> int {
        for (size_t i = 0; i < headers.size(); ++i) if (headers[i] == name) return i;
        return -1;
    };

    int idx_time = get_col_idx("timestamp_s"), idx_throttle_fl = get_col_idx("throttle_fl");
    int idx_throttle_fr = get_col_idx("throttle_fr"), idx_throttle_rl = get_col_idx("throttle_rl");
    int idx_throttle_rr = get_col_idx("throttle_rr"), idx_steering = get_col_idx("steering");
    int idx_x = get_col_idx("real_x"), idx_y = get_col_idx("real_y"), idx_yaw = get_col_idx("real_yaw");
    int idx_vx = get_col_idx("real_vx"), idx_vy = get_col_idx("real_vy"), idx_yaw_rate = get_col_idx("real_yaw_rate");
    int idx_fl_rpm = get_col_idx("real_fl_rpm"), idx_fr_rpm = get_col_idx("real_fr_rpm");
    int idx_rl_rpm = get_col_idx("real_rl_rpm"), idx_rr_rpm = get_col_idx("real_rr_rpm");
    int idx_motor_rpm = get_col_idx("real_motor_rpm");

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::vector<std::string> values;
        std::stringstream line_ss(line);
        while (std::getline(line_ss, cell, ',')) values.push_back(cell);
        if (values.size() < headers.size()) continue;

        auto get_val = [&](int idx, double default_val = 0.0) -> double {
            if (idx < 0 || idx >= static_cast<int>(values.size())) return default_val;
            try { return std::stod(values[idx]); } catch (...) { return default_val; }
        };

        CsvRow row;
        row.timestamp_s = get_val(idx_time); row.throttle_fl = get_val(idx_throttle_fl);
        row.throttle_fr = get_val(idx_throttle_fr); row.throttle_rl = get_val(idx_throttle_rl);
        row.throttle_rr = get_val(idx_throttle_rr); row.steering = get_val(idx_steering);
        row.real_x = get_val(idx_x); row.real_y = get_val(idx_y); row.real_yaw = get_val(idx_yaw);
        row.real_vx = get_val(idx_vx); row.real_vy = get_val(idx_vy); row.real_yaw_rate = get_val(idx_yaw_rate);
        row.real_fl_rpm = get_val(idx_fl_rpm); row.real_fr_rpm = get_val(idx_fr_rpm);
        row.real_rl_rpm = get_val(idx_rl_rpm); row.real_rr_rpm = get_val(idx_rr_rpm);
        row.real_motor_rpm = get_val(idx_motor_rpm);
        rows.push_back(row);
    }
    return rows;
}

std::map<std::string, double*> get_parameter_ptrs(std::shared_ptr<common_lib::car_parameters::CarParameters> p) {
    std::map<std::string, double*> m;
    if (!p) return m;

    m["car.front_bearing_drag"] = &p->front_bearing_drag;
    m["car.wheel_diameter"] = &p->wheel_diameter;
    // ... [Include all your parameter reflection mappings here] ...
    
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
    // ... Deep copy other nested structs as needed ...
    return dst;
}

// ============================================================================
// TEMPLATED EVALUATION: Zero-overhead polymorphic dispatch
// ============================================================================
template <typename ModelType>
double evaluate_candidate(
    const std::vector<std::vector<CsvRow>>& all_csvs_rows,
    const std::vector<ParameterSpec>& param_specs,
    const std::vector<double>& candidate_values,
    const YAML::Node& tuning_config,
    const InvictaSimParameters& base_params
) {
    InvictaSimParameters sim_params = base_params;
    sim_params.car_parameters = deep_copy_car_parameters(base_params.car_parameters);

    for (size_t i = 0; i < param_specs.size(); ++i) {
        apply_parameter_override(sim_params.car_parameters, param_specs[i].name, candidate_values[i]);
    }

    YAML::Node csvs = tuning_config["tuning"]["csvs"];
    YAML::Node default_score_config = tuning_config["tuning"]["score"];

    double weighted_score_sum = 0.0;
    double total_weight = 0.0;

    for (size_t b = 0; b < csvs.size(); ++b) {
        YAML::Node csv_node = csvs[b];
        double weight = csv_node["weight"] ? csv_node["weight"].as<double>() : 1.0;
        double start_offset = csv_node["start_offset_s"] ? csv_node["start_offset_s"].as<double>() : 0.0;
        double stop_duration = csv_node["stop_after_s"] ? csv_node["stop_after_s"].as<double>() : -1.0;

        YAML::Node score_config = default_score_config;
        if (csv_node["score"]) {
            score_config = csv_node["score"];
        }

        const auto& rows = all_csvs_rows[b];
        if (rows.empty()) {
            weighted_score_sum += weight * 1e9;
            total_weight += weight;
            continue;
        }

        size_t start_idx = 0;
        for (size_t i = 0; i < rows.size(); ++i) {
            if (rows[i].timestamp_s >= start_offset) {
                start_idx = i;
                break;
            }
        }

        double start_time = rows[start_idx].timestamp_s;

        // INSTANTIATE THE HARDCODED MODEL DIRECTLY
        ModelType model(sim_params);

        SimState state;
        CsvRow first_sample = rows[start_idx];
        state.x = first_sample.real_x;
        state.y = first_sample.real_y;
        state.yaw = first_sample.real_yaw;
        state.vx = first_sample.real_vx;
        state.vy = first_sample.real_vy;
        state.yaw_rate = first_sample.real_yaw_rate;
        
        // Updated to use the correct wheels_speed struct variables
        state.wheels_speed.front_left = first_sample.real_fl_rpm * 2.0 * M_PI / 60.0;
        state.wheels_speed.front_right = first_sample.real_fr_rpm * 2.0 * M_PI / 60.0;
        state.wheels_speed.rear_left = first_sample.real_rl_rpm * 2.0 * M_PI / 60.0;
        state.wheels_speed.rear_right = first_sample.real_rr_rpm * 2.0 * M_PI / 60.0;
        state.motor_omega = first_sample.real_motor_rpm * 2.0 * M_PI / 60.0;

        RunningRmse position_rmse, heading_rmse, velocity_rmse, velocity_x_rmse, velocity_x_slow_rmse;
        RunningRmse velocity_y_rmse, yaw_rate_rmse, yaw_rate_under_rmse, front_wheel_rpm_rmse, motor_rpm_rmse;

        PoseSample real_origin = {first_sample.real_x, first_sample.real_y, first_sample.real_yaw};
        PoseSample sim_origin = {state.x, state.y, state.yaw};

        double last_time = first_sample.timestamp_s;

        for (size_t i = start_idx + 1; i < rows.size(); ++i) {
            const CsvRow& row = rows[i];
            if (stop_duration > 0.0 && (row.timestamp_s - start_time) > stop_duration) break;
            
            double dt = row.timestamp_s - last_time;
            if (dt <= 0.0) continue;
            last_time = row.timestamp_s;

            const CsvRow& prev_row = rows[i - 1];
            common_lib::structures::Wheels throttle(prev_row.throttle_fl, prev_row.throttle_fr, prev_row.throttle_rl, prev_row.throttle_rr);

            // STEP THE PHYSICS ENGINE
            model.step(dt, throttle, prev_row.steering, state);

            double sim_fl_rpm = state.wheels_speed.front_left * 60.0 / (2.0 * M_PI);
            double sim_fr_rpm = state.wheels_speed.front_right * 60.0 / (2.0 * M_PI);
            double sim_front_rpm = 0.5 * (sim_fl_rpm + sim_fr_rpm);
            double sim_motor_rpm = state.motor_omega * 60.0 / (2.0 * M_PI);
            double real_front_rpm = 0.5 * (row.real_fl_rpm + row.real_fr_rpm);

            PoseSample real_raw = {row.real_x, row.real_y, row.real_yaw};
            PoseSample real_pose = transform_pose_to_map(real_raw, real_origin, sim_origin);

            position_rmse.update(std::hypot(state.x - real_pose.x, state.y - real_pose.y));
            heading_rmse.update(normalize_angle(state.yaw - real_pose.yaw));
            velocity_x_rmse.update(state.vx - row.real_vx);
            velocity_x_slow_rmse.update(std::min(0.0, state.vx - row.real_vx));
            velocity_y_rmse.update(state.vy - row.real_vy);
            velocity_rmse.update(std::hypot(state.vx - row.real_vx, state.vy - row.real_vy));
            yaw_rate_rmse.update(state.yaw_rate - row.real_yaw_rate);
            yaw_rate_under_rmse.update(std::min(0.0, state.yaw_rate - row.real_yaw_rate));
            front_wheel_rpm_rmse.update(sim_front_rpm - real_front_rpm);
            motor_rpm_rmse.update(sim_motor_rpm - row.real_motor_rpm);
        }

        int samples = position_rmse.count;
        double dataset_score = 0.0;
        YAML::Node weights = score_config["weights"];
        if (weights) {
            if (weights["position_rmse"]) dataset_score += weights["position_rmse"].as<double>() * position_rmse.get();
            if (weights["heading_rmse"]) dataset_score += weights["heading_rmse"].as<double>() * heading_rmse.get();
            if (weights["velocity_rmse"]) dataset_score += weights["velocity_rmse"].as<double>() * velocity_rmse.get();
            if (weights["velocity_x_rmse"]) dataset_score += weights["velocity_x_rmse"].as<double>() * velocity_x_rmse.get();
            if (weights["velocity_x_slow_rmse"]) dataset_score += weights["velocity_x_slow_rmse"].as<double>() * velocity_x_slow_rmse.get();
            if (weights["velocity_y_rmse"]) dataset_score += weights["velocity_y_rmse"].as<double>() * velocity_y_rmse.get();
            if (weights["yaw_rate_rmse"]) dataset_score += weights["yaw_rate_rmse"].as<double>() * yaw_rate_rmse.get();
            if (weights["yaw_rate_under_rmse"]) dataset_score += weights["yaw_rate_under_rmse"].as<double>() * yaw_rate_under_rmse.get();
            if (weights["front_wheel_rpm_rmse"]) dataset_score += weights["front_wheel_rpm_rmse"].as<double>() * front_wheel_rpm_rmse.get();
            if (weights["motor_rpm_rmse"]) dataset_score += weights["motor_rpm_rmse"].as<double>() * motor_rpm_rmse.get();
        }

        int min_samples = score_config["min_samples"] ? score_config["min_samples"].as<int>() : 0;
        if (min_samples > 0 && samples < min_samples) {
            double shortfall = static_cast<double>(min_samples - samples) / static_cast<double>(min_samples);
            double penalty = score_config["short_sample_penalty"] ? score_config["short_sample_penalty"].as<double>() : 0.0;
            dataset_score += penalty * shortfall;
        }

        weighted_score_sum += weight * dataset_score;
        total_weight += weight;
    }

    return weighted_score_sum / std::max(total_weight, 1e-9);
}

template <typename ModelType>
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

    std::vector<double> baseline_vals(param_specs.size());
    for (size_t p = 0; p < param_specs.size(); ++p) {
        baseline_vals[p] = get_baseline_value(base_params.car_parameters, param_specs[p].name);
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
    global_best.score = 1e9;

    for (int gen = 0; gen < generations; ++gen) {
        std::cout << "--- Generation " << gen + 1 << "/" << generations << " ---" << std::endl;

        std::vector<std::future<double>> futures(population_size);
        for (int i = 0; i < population_size; ++i) {
            futures[i] = std::async(std::launch::async, [&, i]() {
                // RUNS THE TEMPLATED FUNCTION
                return evaluate_candidate<ModelType>(all_csvs_rows, param_specs, population[i].values, tuning_config, base_params);
            });
        }

        for (int i = 0; i < population_size; ++i) {
            population[i].score = futures[i].get();
            if (population[i].score < global_best.score) {
                global_best = population[i];
                std::cout << "  [New Global Best] Score: " << global_best.score << " Parameters:";
                for (size_t p = 0; p < param_specs.size(); ++p) {
                    std::cout << " " << param_specs[p].name << "=" << global_best.values[p];
                }
                std::cout << std::endl;
            }
        }

        std::sort(population.begin(), population.end(), [](const Individual& a, const Individual& b) {
            return a.score < b.score;
        });

        std::cout << "  Best score in generation: " << population[0].score << std::endl;

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

        auto tournament_select = [&](int size) -> const Individual& {
            int best_idx = rng() % population_size;
            for (int t = 1; t < size; ++t) {
                int idx = rng() % population_size;
                if (population[idx].score < population[best_idx].score) best_idx = idx;
            }
            return population[best_idx];
        };

        while (next_pop.size() < static_cast<size_t>(population_size)) {
            const Individual& parent_a = tournament_select(tournament_size);
            const Individual& parent_b = tournament_select(tournament_size);

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

std::string get_full_csv_path(const std::string& data_dir, const std::string& rel_path) {
    if (std::filesystem::path(rel_path).is_absolute() || data_dir.empty()) {
        return rel_path;
    }
    return (std::filesystem::path(data_dir) / rel_path).string();
}

int main(int argc, char** argv) {
    std::string config_path;
    std::string data_dir;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--data-dir" && i + 1 < argc) {
            data_dir = argv[++i];
        }
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
    
    // IMPORTANT: Make sure you load your baseline car parameters here!
    // For example: base_params.car_parameters = config::load_car_params("car.yaml");

    YAML::Node csvs = tuning_config["tuning"]["csvs"];
    std::vector<std::vector<CsvRow>> all_csvs_rows;
    all_csvs_rows.reserve(csvs.size());

    std::cout << "Loading CSV telemetry datasets..." << std::endl;
    for (size_t b = 0; b < csvs.size(); ++b) {
        std::string rel_path = csvs[b]["path"].as<std::string>();
        std::string full_path = get_full_csv_path(data_dir, rel_path);
        std::cout << "  Reading: " << full_path << std::endl;
        auto rows = read_csv(full_path);
        std::cout << "    Loaded " << rows.size() << " samples." << std::endl;
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
    
    // Extract the vehicle model from the tuning YAML
    std::string requested_model = "fsfeup02"; // default
    if (tuning_config["tuning"]["vehicle_model"]) {
        requested_model = tuning_config["tuning"]["vehicle_model"].as<std::string>();
    }
    std::cout << "Selected vehicle model from configuration: " << requested_model << std::endl;

    std::cout << "Starting Genetic Algorithm search..." << std::endl;
    auto start_time = std::chrono::steady_clock::now();
    
    Individual optimal;

    // ====================================================================
    // MODEL ROUTER: Add future inline models here!
    // ====================================================================
    if (requested_model == "fsfeup02" || requested_model == "inline_02") {
        optimal = run_genetic_algorithm<InlineFSFEUP02Model>(all_csvs_rows, param_specs, tuning_config, base_params);
    } 
    /*
    else if (requested_model == "fsfeup03" || requested_model == "inline_03") {
        optimal = run_genetic_algorithm<InlineFSFEUP03Model>(all_csvs_rows, param_specs, tuning_config, base_params);
    }
    */
    else {
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