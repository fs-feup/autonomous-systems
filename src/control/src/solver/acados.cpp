#include "solver/acados/acados.hpp"

#include <cmath>
#include <limits>

constexpr int kPathSize = 31;
constexpr int kPathPointSize = 4;
constexpr double kWeightEps = 0.5;

AcadosSolver::AcadosSolver(const ControlParameters& params) : SolverInterface(params), _execution_times_(std::make_shared<std::vector<double>>(9, 0.0)) {
    // 1. Create the capsule
    this->capsule_ = mpc_acados_create_capsule();
    
    // 2. Allocate solver memory
    int status = mpc_acados_create(this->capsule_);
    if (status != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("AcadosSolver"), "Failed to create Acados solver 'mpc', status: %d", status);
    }

    // 3. Cache internal pointers
    nlp_config_ = mpc_acados_get_nlp_config(this->capsule_);
    nlp_dims_ = mpc_acados_get_nlp_dims(this->capsule_);
    nlp_in_ = mpc_acados_get_nlp_in(this->capsule_);
    nlp_out_ = mpc_acados_get_nlp_out(this->capsule_);
}

AcadosSolver::~AcadosSolver() {
    mpc_acados_free(this->capsule_);
    mpc_acados_free_capsule(this->capsule_);
}

void AcadosSolver::set_state(const std::vector<double>& x0) {
    if (x0.size() != 13) {
        RCLCPP_WARN(rclcpp::get_logger("AcadosSolver"), "State size mismatch: expected 13, got %zu", x0.size());
        return;
    }

    this->last_state_ = x0;
    this->has_state_ = true;

    // Set the initial state constraint (lbx and ubx) at stage 0
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", (void*)x0.data());
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", (void*)x0.data());
}

void AcadosSolver::set_path_point_per_stage() {
    unsigned int path_point_count = this->last_path_.size() / kPathPointSize;

    // Get the horizon length and time step
    int N = this->control_params_->mpc_prediction_horizon_steps_;
    double time_step = this->control_params_->mpc_prediction_horizon_seconds_ / static_cast<double>(N);
    //std::cout << "Horizon length (N): " << N << ", Time step: " << time_step << " seconds" << std::endl;

    // Find the closest point on the line segment between first and second points
    double first_x = this->last_path_[0];
    double first_y = this->last_path_[1];
    double first_v = this->last_path_[2];
    double first_orientation = this->last_path_[3];
    double first_point_time = 0; // Can be the time of a path point or stage point
  
    double second_x = this->last_path_[kPathPointSize];
    double second_y = this->last_path_[kPathPointSize + 1];
    double second_v = this->last_path_[kPathPointSize + 2];
    double second_orientation = this->last_path_[kPathPointSize + 3];

    // Find the closest point on the line segment between first and second points
    double car_x = this->last_state_[0];
    double car_y = this->last_state_[1];
    double dx = second_x - first_x;
    double dy = second_y - first_y;
    double segment_length_sq = dx * dx + dy * dy;
    double t = 0.0;
    if (segment_length_sq > 1e-6) {
      t = ((car_x - first_x) * dx + (car_y - first_y) * dy) / segment_length_sq;
      t = std::max(0.0, std::min(1.0, t));
    }

    // First point is the closest
    double path_point_x = first_x + t * dx;
    double path_point_y = first_y + t * dy;
    double path_point_v = (1-t) * first_v + t * (second_v);
    double path_point_orientation = (1-t) * first_orientation + t * (second_orientation);
    double path_point_time = 0;

    double average_velocity = (path_point_v + second_v) / 2.0;
    double second_point_time = (1-t)*std::sqrt(segment_length_sq)/(average_velocity + 1e-6); // Assumes constant acceleration

    unsigned int segment_end_index = 1;

    // Iterate over all stages (0 to N) to set the time-varying parameters
    // std::string path = "path: ";
    for (int i = 0; i <= N; ++i) {
      // path += "(" + std::to_string(path_point_x) + ", " + std::to_string(path_point_y) + "), ";
      double point_for_stage[4] = {path_point_x, path_point_y, path_point_v, path_point_orientation};
      mpc_acados_update_params(this->capsule_, i, point_for_stage, kPathPointSize);

      while (second_point_time - path_point_time < time_step && segment_end_index < path_point_count - 1) {
        segment_end_index++;

        first_x = second_x;
        first_y = second_y;
        first_v = second_v;
        first_orientation = second_orientation;
        first_point_time = second_point_time;

        second_x = this->last_path_[segment_end_index * kPathPointSize];
        second_y = this->last_path_[segment_end_index * kPathPointSize + 1];
        second_v = this->last_path_[segment_end_index * kPathPointSize + 2];
        second_orientation = this->last_path_[segment_end_index * kPathPointSize + 3];

        dx = second_x - first_x;
        dy = second_y - first_y;
        segment_length_sq = dx * dx + dy * dy;

        average_velocity = (first_v + second_v) / 2.0;
        second_point_time += std::sqrt(segment_length_sq)/(average_velocity + 1e-6);
      }

      double time_spent_on_segment = time_step - (first_point_time - path_point_time);
      double segment_delta_time = second_point_time - first_point_time;
      t = time_spent_on_segment / segment_delta_time;

      dx = second_x - first_x;
      dy = second_y - first_y;

      // First point is the closest
      path_point_x = first_x + t * dx;
      path_point_y = first_y + t * dy;
      path_point_v = (1-t) * first_v + t * (second_v);
      path_point_orientation = (1-t) * first_orientation + t * (second_orientation);
      path_point_time += time_step;

      first_x = path_point_x;
      first_y = path_point_y;
      first_v = path_point_v;
      first_orientation = path_point_orientation;
      first_point_time = path_point_time;
    }
    // std::cout << path << std::endl;
}

void AcadosSolver::update_execution_times() {
  // Create temporary variables to receive the raw values
  double t_tot, t_lin, t_sim, t_qp, t_reg;
  int sqp_iter;
  // Get the values from Acados (Times are in Seconds, Iterations is Int)
  ocp_nlp_solver *nlp_solver = mpc_acados_get_nlp_solver(this->capsule_);
  ocp_nlp_get(nlp_solver, "time_tot", &t_tot);
  ocp_nlp_get(nlp_solver, "time_lin", &t_lin);
  ocp_nlp_get(nlp_solver, "time_sim", &t_sim);
  ocp_nlp_get(nlp_solver, "time_qp",  &t_qp);
  ocp_nlp_get(nlp_solver, "time_reg", &t_reg);
  ocp_nlp_get(nlp_solver, "sqp_iter", &sqp_iter);
  // Convert times to milliseconds
  t_tot = t_tot * 1000.0;
  t_lin = t_lin * 1000.0;
  t_sim = t_sim * 1000.0;
  t_qp  = t_qp  * 1000.0;
  t_reg = t_reg * 1000.0;
  // Store in execution times vector for visualization
  (*_execution_times_)[0] = t_tot;
  (*_execution_times_)[1] = t_lin;
  (*_execution_times_)[2] = t_sim;
  (*_execution_times_)[3] = t_qp ;
  (*_execution_times_)[4] = t_reg;
  // Convert iteration count to double for easier visualization, even though it's an integer
  (*_execution_times_)[5] = (double)sqp_iter;
  // Update averages
  this->average_linearization_time_ = ((this->average_linearization_time_ * static_cast<double>(this->linearization_count_)) + t_lin) / static_cast<double>(this->linearization_count_ + 1);
  this->linearization_count_++;
  this->average_qp_time_ = ((this->average_qp_time_ * this->qp_count_) + t_qp) / static_cast<double>(this->qp_count_ + 1);   
  this->qp_count_++;
  this->average_regularization_time_ = ((this->average_regularization_time_ * this->regularization_count_) + t_reg) / static_cast<double>(this->regularization_count_ + 1);
  this->regularization_count_++;
  (*_execution_times_)[6] = average_linearization_time_;
  (*_execution_times_)[7] = average_qp_time_;
  (*_execution_times_)[8] = average_regularization_time_;
}

void AcadosSolver::set_path(const std::vector<double>& x_path) {
    // const int stage_count = this->control_params_->mpc_prediction_horizon_steps_ + 1;
    // const size_t required_path_size = static_cast<size_t>(stage_count * kPathPointSize);
// 
    // if (x_path.size() < required_path_size) {
    //   RCLCPP_WARN(rclcpp::get_logger("AcadosSolver"), "Path too small");
    //   return;
    // }
    // RCLCPP_INFO(rclcpp::get_logger("AcadosSolver"), "Received path with %zu points", x_path.size() / kPathPointSize);

    this->last_path_ = x_path;
    this->has_path_ = true;

    // for (int stage = 0; stage < stage_count; ++stage) {
    //  const double point_for_stage[kPathPointSize] = {
    //    x_path[stage * kPathPointSize],
    //    x_path[stage * kPathPointSize + 1],
    //    x_path[stage * kPathPointSize + 2],
    //    x_path[stage * kPathPointSize + 3]
    //  };
    //  mpc_acados_update_params(capsule_, stage, const_cast<double*>(point_for_stage), kPathPointSize);
    // }

  
    // std::string received_path = "";
    // std::string received_path_info = "Info:";
    // for (int i = 0; i < kPathPointSize * (x_path.size() > 50 ? 50 : x_path.size()); i += kPathPointSize) {
    //   received_path += "(" + std::to_string(x_path[i]) + ", " + std::to_string(x_path[i+1]) + "), ";
    //   received_path_info += "(" + std::to_string(x_path[i]) + ", " + std::to_string(x_path[i+1]) + "," + std::to_string(x_path[i+2]) + "," + std::to_string(x_path[i+3]) + "), ";
    // }
    // std::cout << "Received path points: " << received_path << std::endl;
    // std::cout << received_path_info << std::endl;
}

common_lib::structures::ControlCommand AcadosSolver::solve() {
  common_lib::structures::ControlCommand command;
  if (!(this->has_state_ && this->has_path_)) {
    return command;
  }

  this->set_path_point_per_stage();

  int status = mpc_acados_solve(this->capsule_);
  if (status != ACADOS_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("AcadosSolver"), "Acados solver failed with status %d", status);
  }
  this->update_execution_times();
  // Extracting 3 controls
  double solved_controls[3]; 
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", (void*)solved_controls);
  // Map the controls to our command
  command.throttle_rl= solved_controls[0];
  command.throttle_rr= solved_controls[1];
  command.steering_angle = solved_controls[2];
  return command;
}

std::vector<common_lib::structures::ControlCommand> AcadosSolver::get_full_solution() {
  int N = nlp_dims_->N;
  std::vector<common_lib::structures::ControlCommand> full_u;
  full_u.reserve(N);
  
  for (int i = 0; i < N; ++i) {
    double solved_controls[3]; 
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "u", (void*)solved_controls);

    common_lib::structures::ControlCommand cmd;
    cmd.throttle_rl= solved_controls[0]; 
    cmd.throttle_rr= solved_controls[1];
    cmd.steering_angle = solved_controls[2];
    full_u.push_back(cmd);
  }
  return full_u;
}

std::vector<custom_interfaces::msg::VehicleStateVector> AcadosSolver::get_full_horizon() {
  int N = nlp_dims_->N;
  constexpr int kStateSize = 13;
  std::vector<custom_interfaces::msg::VehicleStateVector> full_horizon;
  full_horizon.reserve(N + 1);
  for (int i = 0; i <= N; ++i) {
    double solved_state[kStateSize] = {0.0};
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", (void*)solved_state);
    custom_interfaces::msg::VehicleStateVector state_vector;
    state_vector.velocity_x = solved_state[3];
    state_vector.velocity_y = solved_state[4];
    state_vector.yaw_rate = solved_state[5];
    state_vector.acceleration_x = solved_state[6];
    state_vector.acceleration_y = solved_state[7];
    state_vector.steering_angle = solved_state[8];
    state_vector.fl_rpm = solved_state[9];
    state_vector.fr_rpm = solved_state[10];
    state_vector.rl_rpm = solved_state[11];
    state_vector.rr_rpm = solved_state[12];
    full_horizon.emplace_back(state_vector);
  }
  return full_horizon;
}

void AcadosSolver::publish_solver_data(std::shared_ptr<rclcpp::Node> node, std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>>& publisher_map) {
  if (publisher_map.find("/acados/execution_times") == publisher_map.end()) {
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/acados/execution_times", 10);
    publisher_map["/acados/execution_times"] = publisher;
  }

  auto publisher = std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>>(publisher_map["/acados/execution_times"]);
  std_msgs::msg::Float64MultiArray msg;
  msg.data = *this->_execution_times_;
  publisher->publish(msg);
}