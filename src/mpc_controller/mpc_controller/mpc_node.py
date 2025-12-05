import math
import os
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import yaml
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from casadi import SX, vertcat, sin, cos, atan, if_else
from rclpy.node import Node
import rclpy

from custom_interfaces.msg import ControlCommand, OperationalStatus, PathPointArray, Pose, Velocities
from pacsim.msg import StampedScalar
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# --- Helper Functions ---

def _load_yaml(path: Path) -> dict:
    with path.open("r") as handle:
        return yaml.safe_load(handle)


def _find_config_root() -> Path:
    env_override = os.environ.get("MPC_CONFIG_ROOT")
    if env_override:
        candidate = Path(env_override).expanduser()
        if (candidate / "global" / "global_config.yaml").exists():
            return candidate
    current = Path(__file__).resolve()
    for parent in [current] + list(current.parents):
        cfg = parent / "config"
        if (cfg / "global" / "global_config.yaml").exists():
            return cfg
    return Path.cwd() / "config"


def _wrap_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi


# --- Classes ---

class LongitudinalPID:
    """Optimized PI-D controller (kept for reference, not used in full MPC)."""
    def __init__(self, params: dict):
        self.kp = float(params.get("pid_kp", 0.1))
        self.ki = float(params.get("pid_ki", 0.0))
        self.kd = float(params.get("pid_kd", 0.0))
        self.dt = float(params.get("pid_t", 0.03))
        self.lim_min = float(params.get("pid_lim_min", -1.0))
        self.lim_max = float(params.get("pid_lim_max", 1.0))

        self.integrator = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0

    def update(self, setpoint: float, measurement: float) -> float:
        error = setpoint - measurement

        # Proportional
        P = self.kp * error

        # Integral (Trapezoidal) with clamping
        self.integrator += 0.5 * self.ki * self.dt * (error + self.prev_error)
        self.integrator = max(self.lim_min, min(self.lim_max, self.integrator))

        # Derivative (on measurement to avoid kick)
        D = -self.kd * (measurement - self.prev_measurement) / self.dt

        out = P + self.integrator + D
        out = max(self.lim_min, min(self.lim_max, out))

        self.prev_error = error
        self.prev_measurement = measurement
        return out


class LateralMPC:
    """Full (lateral + longitudinal) MPC with Dynamic Bicycle model and motor/power model."""

    def __init__(self, car_params: dict, control_params: dict):
        self.model_name = "fs_full_bicycle"

        # Car Parameters
        self.m = float(car_params.get("mass", 150.0))
        self.Iz = float(car_params.get("Izz", 80.0))

        wb = float(car_params.get("wheel_base", 1.53))
        dist_cg_r = float(car_params.get("dist_cg_2_rear_axis", 0.8))
        self.Lr = dist_cg_r
        self.Lf = wb - self.Lr

        # Aero & drag
        self.Cd = float(car_params.get("drag_coefficient", 0.7))
        self.Cl = float(car_params.get("lift_coefficient", -1.0))  # Negative for downforce
        self.A_front = float(car_params.get("frontal_area", 1.0))
        self.rho = 1.225
        self.C_roll = float(car_params.get("rolling_resistance", 0.015))

        # Wheel & powertrain
        self.Rw = float(car_params.get("wheel_radius", 0.25))  # m, default ~0.25
        self.gear_ratio = float(car_params.get("gear_ratio", 4.0))
        # From your description:
        # Motor peak torque = 220 Nm, gear ratio = 4 -> ~880 Nm at wheels,
        # but you want to clamp at 800 Nm.
        self.tau_wheel_max = 800.0  # Nm at the wheels
        self.P_max = 80e3           # 80 kW

        # Pacejka (magnitude coefficients, signs handled in equations)
        self.D_tire = float(car_params.get("tire_D_lateral", 1.6))
        self.C_tire = abs(float(car_params.get("tire_C_lateral", 1.39)))
        self.B_tire = float(car_params.get("tire_B_lateral", 9.0))

        # Control Constraints
        self.delta_max = float(control_params.get("steering_max", 0.35))
        self.delta_min = float(control_params.get("steering_min", -0.35))
        self.delta_rate_max = float(control_params.get("mpc_max_steer_rate", 1.5))

        # Horizon
        self.N = int(control_params.get("mpc_horizon_steps", 20))
        self.dt = float(control_params.get("mpc_dt", 0.05))
        self.w_progress = float(control_params.get("mpc_w_progress", 0.0))        
        
        # Weights (now with vx and throttle)
        self.Q_diag = [
            control_params.get("mpc_w_ey", 80.0),      # ey
            control_params.get("mpc_w_epsi", 30.0),    # epsi
            control_params.get("mpc_w_vx", 10.0),      # vx
            control_params.get("mpc_w_vy", 5.0),       # vy
            control_params.get("mpc_w_r", 2.0),        # r
            control_params.get("mpc_w_delta", 0.5),    # delta
        ]
        self.R_diag = [
            control_params.get("mpc_w_d_delta", 1.0),   # steering rate
            control_params.get("mpc_w_throttle", 1.0),  # normalized throttle
        ]

        self.last_delta = 0.0
        self._setup_acados()

    def _setup_acados(self):
        # 1. Define Model
        model = AcadosModel()
        model.name = self.model_name

        # States: [ey, epsi, vx, vy, r, delta]
        ey = SX.sym("ey")
        epsi = SX.sym("epsi")
        vx = SX.sym("vx")
        vy = SX.sym("vy")
        r = SX.sym("r")
        delta = SX.sym("delta")
        x = vertcat(ey, epsi, vx, vy, r, delta)

        # Controls: [steering_rate, throttle_command]
        ddelta = SX.sym("ddelta")
        uthrottle = SX.sym("uthrottle")  # in [-1, 1]
        u = vertcat(ddelta, uthrottle)

        # Parameters: [curvature, v_ref]
        p = SX.sym("p", 2)
        p_k = p[0]       # curvature
        v_ref = p[1]     # reference longitudinal speed along the path

        # --- Dynamics ---
        # Safe vx for divisions
        vx_safe = if_else(vx < 1.0, 1.0, vx)

        # Slip angles
        alpha_f = delta - atan((vy + self.Lf * r) / vx_safe)
        alpha_r = -atan((vy - self.Lr * r) / vx_safe)

        # Vertical Loads (Aero uses vx)
        Fz_static = self.m * 9.81
        Fz_aero = 0.5 * self.rho * self.Cl * self.A_front * vx * vx
        Fz_tot = Fz_static - Fz_aero
        Fz_f = (self.Lr / (self.Lf + self.Lr)) * Fz_tot
        Fz_r = (self.Lf / (self.Lf + self.Lr)) * Fz_tot

        # Pacejka lateral forces
        Fy_f = Fz_f * self.D_tire * sin(self.C_tire * atan(self.B_tire * alpha_f))
        Fy_r = Fz_r * self.D_tire * sin(self.C_tire * atan(self.B_tire * alpha_r))

        # Longitudinal forces: motor & drag & rolling resistance
        tau_cmd = uthrottle * self.tau_wheel_max  # wheel torque command

        # Absolute torque and wheel speed
        tau_abs = if_else(tau_cmd >= 0, tau_cmd, -tau_cmd)
        w_wheel = vx / self.Rw
        w_abs = if_else(w_wheel >= 0, w_wheel, -w_wheel)

        P_cmd = tau_abs * w_abs
        eps = 1e-3

        # Power-limited torque magnitude
        tau_allowed = if_else(
            P_cmd > self.P_max,
            self.P_max / (w_abs + eps),
            tau_abs,
        )
        sign_tau = if_else(tau_cmd >= 0, 1.0, -1.0)
        tau_final = sign_tau * tau_allowed

        Fx_drive = tau_final / self.Rw

        # Aerodynamic drag and rolling resistance
        sign_vx = if_else(vx >= 0, 1.0, -1.0)
        F_drag = 0.5 * self.rho * self.Cd * self.A_front * vx * vx * sign_vx
        F_roll = self.C_roll * self.m * 9.81 * sign_vx

        # State Derivatives (curvilinear coordinates)
        dey = vy * cos(epsi) + vx * sin(epsi)
        depsi = r - p_k * vx
        dvx = (Fx_drive - F_drag - F_roll) / self.m + r * vy
        dvy = (Fy_f * cos(delta) + Fy_r) / self.m - vx * r
        dr = (self.Lf * Fy_f * cos(delta) - self.Lr * Fy_r) / self.Iz
        ddelta_dt = ddelta

        model.f_expl_expr = vertcat(dey, depsi, dvx, dvy, dr, ddelta_dt)
        model.x = x
        model.u = u
        model.p = p

        # 2. Define OCP
        ocp = AcadosOcp()
        ocp.model = model

        # Parameters
        ocp.parameter_values = np.zeros(2)

        # Horizon
        ocp.solver_options.N_horizon = self.N

        nx = 6
        nu = 2

        # --- EXTERNAL COST: tracking + lap-time progress reward ---

        w_ey, w_epsi, w_vx, w_vy, w_r, w_delta = self.Q_diag
        w_ddelta, w_uthrottle = self.R_diag
        w_prog = self.w_progress

        # Progress speed along path (projection of velocity onto path tangent)
        v_prog = vx * cos(epsi) - vy * sin(epsi)

        # Desired yaw rate from curvature and reference speed
        r_ref = v_ref * p_k

        # Running cost
        ell = (
            w_ey * ey**2 +
            w_epsi * epsi**2 +
            w_vx * (vx - v_ref)**2 +
            w_vy * vy**2 +
            w_r * (r - r_ref)**2 +
            w_delta * delta**2 +
            w_ddelta * ddelta**2 +
            w_uthrottle * uthrottle**2
            - w_prog * v_prog         # <-- LAP-TIME TERM (reward progress)
        )

        # Terminal cost (no input terms; you can keep progress reward here too or drop it)
        phi = (
            w_ey * ey**2 +
            w_epsi * epsi**2 +
            w_vx * (vx - v_ref)**2 +
            w_vy * vy**2 +
            w_r * (r - r_ref)**2 +
            w_delta * delta**2
            - w_prog * v_prog
        )

        ocp.model.cost_expr_ext_cost = ell
        ocp.model.cost_expr_ext_cost_e = phi

        ocp.cost.cost_type = "EXTERNAL"
        ocp.cost.cost_type_e = "EXTERNAL"


        # Initial state constraint (placeholder; actual x0 set at runtime)
        ocp.constraints.x0 = np.zeros(nx)

        ###############################################
        # HARD CONSTRAINT: steering angle only
        ###############################################
        ocp.constraints.lbx = np.array([self.delta_min])
        ocp.constraints.ubx = np.array([self.delta_max])
        ocp.constraints.idxbx = np.array([5])   

        ###############################################
        # SOFT CONSTRAINT: ey
        ###############################################
        # C * x + D * u
        ocp.constraints.C = np.array([
            [ 1, 0, 0, 0, 0, 0],   # ey
            [-1, 0, 0, 0, 0, 0]    # -ey
        ])

        # ---> FIX: Define D matrix (2 constraints, 2 controls) <---
        ocp.constraints.D = np.zeros((2, nu))

        big = 1e9
        ocp.constraints.lg = np.array([-1.0, -1.0])
        ocp.constraints.ug = np.array([ big,  big])

        # we soften BOTH general constraints (indices 0 and 1 of C/D matrices)
        ocp.constraints.idxsg = np.array([0, 1])

        # Slack penalty weights (L2 + L1)
        soft_L2 = 200.0
        soft_L1 = 10.0

        ocp.cost.Zl = soft_L2 * np.ones(2)
        ocp.cost.Zu = soft_L2 * np.ones(2)
        ocp.cost.zl = soft_L1 * np.ones(2)
        ocp.cost.zu = soft_L1 * np.ones(2)

        # Control bounds: steering rate and throttle
        ocp.constraints.lbu = np.array([-self.delta_rate_max, -1.0])
        ocp.constraints.ubu = np.array([self.delta_rate_max, 1.0])
        ocp.constraints.idxbu = np.array([0, 1])

        # Solver Options
        ocp.solver_options.tf = self.N * self.dt
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 1

        # Code Gen Paths
        home = Path.home()
        cache_dir = home / ".cache" / "mpc_controller_acados"
        cache_dir.mkdir(parents=True, exist_ok=True)

        ocp.code_export_directory = str(cache_dir / "c_generated_code")
        json_file = str(cache_dir / "acados_ocp.json")

        self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        

    def solve(self, x0, vx_ref_profile, k_ref_profile):
        # x0: [ey, epsi, vx, vy, r, delta]

        # Set Initial State as bounds at stage 0
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)

        # Set parameters and stage references
                # Set parameters for each stage: [kappa, v_ref]
        for i in range(self.N):
            v_ref = float(vx_ref_profile[i])
            k = float(k_ref_profile[i])

            # Params: [kappa, v_ref]
            self.solver.set(i, "p", np.array([k, v_ref]))


        # Terminal stage parameters
        v_ref_N = float(vx_ref_profile[self.N])
        k_N = float(k_ref_profile[self.N])
        self.solver.set(self.N, "p", np.array([k_N, v_ref_N]))


        # Solve
        status = self.solver.solve()

        if status != 0:
            print(f"[MPC ERROR] Acados solver failed. Status: {status}")
            # Keep last steering, zero throttle
            return self.last_delta, 0.0, []

        # Get Control
        u0 = self.solver.get(0, "u")
        d_delta = u0[0]
        u_throttle = u0[1]

        self.last_delta = x0[5] + d_delta * self.dt

        # Get Predictions for viz
        preds = []
        for i in range(self.N + 1):
            preds.append(self.solver.get(i, "x"))

        return self.last_delta, u_throttle, preds


class MpcControllerNode(Node):
    def __init__(self):
        super().__init__("mpc_controller")

        # Config
        self.config_root = _find_config_root()
        self.global_config = _load_yaml(self.config_root / "global" / "global_config.yaml")["global"]
        self.adapter = self.global_config.get("adapter", "vehicle")

        control_cfg = _load_yaml(self.config_root / "control" / f"{self.adapter}.yaml")["control"]
        car_cfg = _load_yaml(self.config_root / "car" / f"{self.adapter}.yaml")["car"]

        self.frame_id = self.global_config.get("vehicle_frame_id", "map")
        self.use_sim = self.global_config.get("use_simulated_planning", False)

        # Topics
        self.path_topic = "/path_planning/mock_path" if self.use_sim else "/path_planning/path"

        # Controllers
        # Full MPC handles both longitudinal and lateral now
        self.mpc = LateralMPC(car_cfg, control_cfg)

        # State
        self.path_np = None
        self.path_s = None
        self.current_pose = None
        self.current_vel = None
        self.go_signal = True if self.adapter == "pacsim" else False
        self.path_heading = None
        self.horizon_s = None
        self.track_length = None
        self.last_s_unwrapped = None

        # --- SUBSCRIPTIONS ---
        self.create_subscription(PathPointArray, self.path_topic, self._path_cb, 1)

        if self.adapter == "pacsim":
            from geometry_msgs.msg import TwistWithCovarianceStamped
            # Pacsim uses TwistWithCovarianceStamped for both Pose and Velocity
            self.create_subscription(TwistWithCovarianceStamped, "/pacsim/pose", self._pacsim_pose_cb, 1)
            self.create_subscription(TwistWithCovarianceStamped, "/pacsim/velocity", self._pacsim_vel_cb, 1)
        else:
            self.create_subscription(Pose, "/state_estimation/vehicle_pose", self._pose_cb, 1)
            self.create_subscription(Velocities, "/state_estimation/velocities", self._vel_cb, 1)
            self.create_subscription(OperationalStatus, "/vehicle/operational_status", self._op_status_cb, 1)

        # Publishers
        self.control_pub = self.create_publisher(ControlCommand, "/control/command", 1)
        if self.adapter == "pacsim":
            self.debug_steer = self.create_publisher(StampedScalar, "/pacsim/steering_setpoint", 1)
            self.debug_acc = self.create_publisher(StampedScalar, "/pacsim/throttle_setpoint", 1)

        self.viz_pub = self.create_publisher(MarkerArray, "/control/mpc_pred", 1)
        self.exec_time_pub = self.create_publisher(Float64MultiArray, "/control/execution_time", 1)

        # Timer
        self.timer_period = float(control_cfg.get("command_time_interval", 20)) / 1000.0
        self.timer = self.create_timer(self.timer_period, self._control_loop)

        self.get_logger().info(f"Full MPC (lat + lon) Ready. Adapter: {self.adapter}")

    # --- Callbacks ---

    def _pacsim_pose_cb(self, msg):
        from custom_interfaces.msg import Pose as PoseMsg
        p = PoseMsg()
        p.x = msg.twist.twist.linear.x
        p.y = msg.twist.twist.linear.y
        p.theta = msg.twist.twist.angular.z
        self.current_pose = p

    def _pacsim_vel_cb(self, msg):
        from custom_interfaces.msg import Velocities as VelMsg
        v = VelMsg()
        v.velocity_x = msg.twist.twist.linear.x
        v.velocity_y = msg.twist.twist.linear.y
        v.angular_velocity = msg.twist.twist.angular.z
        self.current_vel = v

    def _op_status_cb(self, msg: OperationalStatus):
        self.go_signal = msg.go_signal

    def _pose_cb(self, msg: Pose):
        self.current_pose = msg

    def _vel_cb(self, msg: Velocities):
        self.current_vel = msg

    def _path_cb(self, msg: PathPointArray):
        if not msg.pathpoint_array:
            return

        n = len(msg.pathpoint_array)
        data = np.zeros((n, 4))  # x, y, v, kappa

        for i, p in enumerate(msg.pathpoint_array):
            data[i, 0] = p.x
            data[i, 1] = p.y
            data[i, 2] = p.v
            data[i, 3] = 0.0

        # Curvature
        if n > 2:
            # Check if closed loop
            is_closed = False
            if n > 5:
                first = data[0, :2]
                last = data[-1, :2]
                if np.linalg.norm(first - last) < 1.0: # Threshold for closed loop
                    is_closed = True

            if is_closed:
                # Pad for continuity
                pad = 5
                pre = data[-pad-1:-1, :2] # Take last few points (excluding the very last duplicate if it exists, but here we just take last few)
                post = data[1:pad+1, :2]  # Take first few points
                
                # Actually, better to just use the raw points. 
                # If data[0] == data[-1], we should be careful.
                # Let's assume standard closed loop where last point ~= first point.
                
                # Construct augmented path for curvature calc
                # Prepend last 'pad' points, Append first 'pad' points
                pts = data[:, :2]
                pts_aug = np.vstack([pts[-pad-1:-1], pts, pts[1:pad+1]])
            else:
                pts_aug = data[:, :2]

            # Calculate curvature on augmented path
            p_m = pts_aug[:-2]
            p = pts_aug[1:-1]
            p_p = pts_aug[2:]

            a = np.linalg.norm(p - p_m, axis=1)
            b = np.linalg.norm(p_p - p, axis=1)
            c = np.linalg.norm(p_p - p_m, axis=1)

            s_tri = (a + b + c) / 2
            area = np.sqrt(np.abs(s_tri * (s_tri - a) * (s_tri - b) * (s_tri - c)))
            kappa_aug = 4 * area / (a * b * c + 1e-6)

            v1 = p - p_m
            v2 = p_p - p
            cross = v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0]
            kappa_aug = np.where(cross < 0, -kappa_aug, kappa_aug)

            if is_closed:
                # Extract valid region
                # We added 'pad' points at start. 
                # kappa_aug has length len(pts_aug) - 2.
                # The first valid curvature corresponds to index 'pad' in pts_aug (which is data[0])
                # In kappa_aug, this is index 'pad - 1'.
                pad = 5
                valid_kappa = kappa_aug[pad-1 : pad-1 + n]
                
                # Assign
                data[:, 3] = valid_kappa
                
                # Ensure exact continuity at boundaries for closed loop
                k_mean = (data[0, 3] + data[-1, 3]) / 2.0
                data[0, 3] = k_mean
                data[-1, 3] = k_mean
                
            else:
                # Standard open loop handling
                data[1:-1, 3] = kappa_aug
                data[0, 3] = kappa_aug[0]
                data[-1, 3] = kappa_aug[-1]

        # Centerline arc-length
        deltas = np.diff(data[:, :2], axis=0)
        dists = np.sqrt((deltas ** 2).sum(axis=1))
        s = np.concatenate(([0], np.cumsum(dists)))

        # Save base arrays
        self.path_np = data
        self.path_s = s
        self.track_length = float(s[-1])

        # Compute heading FIRST
        n = data.shape[0]
        headings = np.zeros(n)
        headings[:-1] = np.arctan2(deltas[:, 1], deltas[:, 0])
        headings[-1] = headings[-2]
        
        # Smooth heading for closed loop
        if n > 2 and np.linalg.norm(data[0, :2] - data[-1, :2]) < 1.0:
             # Average start and end heading (handling wrap)
             h_start = headings[0]
             h_end = headings[-1]
             # Vector average
             vx = (np.cos(h_start) + np.cos(h_end)) / 2
             vy = (np.sin(h_start) + np.sin(h_end)) / 2
             h_avg = np.arctan2(vy, vx)
             headings[0] = h_avg
             headings[-1] = h_avg

        self.path_heading = headings

        # --- NOW SAFE TO CREATE PERIODIC EXTENDED ARRAYS ---
        # Triple buffer to be safe for long horizons or high speeds near wrap
        self.path_s_extended = np.concatenate([self.path_s, self.path_s + self.track_length, self.path_s + 2*self.track_length])
        self.path_np_extended = np.vstack([self.path_np, self.path_np, self.path_np])
        self.path_heading_extended = np.concatenate([self.path_heading, self.path_heading, self.path_heading])

    # --- Control Loop ---

    def _control_loop(self):
        t0 = self.get_clock().now()

        missing = []
        if not self.go_signal:
            missing.append("Go Signal")
        if self.current_pose is None:
            missing.append("Pose")
        if self.current_vel is None:
            missing.append("Velocity")
        if self.path_np is None:
            missing.append("Path")

        if missing:
            self.get_logger().info(f"Waiting for: {', '.join(missing)}", throttle_duration_sec=2.0)
            return

        # 1. State Extraction
        px, py = self.current_pose.x, self.current_pose.y
        theta = self.current_pose.theta

        vx = self.current_vel.velocity_x
        vy = self.current_vel.velocity_y
        r = self.current_vel.angular_velocity

        if abs(vx) < 0.1:
            vx = 0.1  # Singularity protection

                # 2. Fast Projection
        dx = self.path_np[:, 0] - px
        dy = self.path_np[:, 1] - py
        dists_sq = dx ** 2 + dy ** 2
        idx_min = int(np.argmin(dists_sq))

        closest_x = self.path_np[idx_min, 0]
        closest_y = self.path_np[idx_min, 1]

        # Use precomputed, smoothed heading at closest point
        path_heading = float(self.path_heading[idx_min])

        ey = -math.sin(path_heading) * (px - closest_x) + math.cos(path_heading) * (py - closest_y)
        epsi = _wrap_angle(theta - path_heading)

        ref_v = float(self.path_np[idx_min, 2])

        # 3. Reference Generation (curvilinear s with lap-unwrapping)
        s_raw = float(self.path_s[idx_min])   # in [0, track_length]
        L = float(self.track_length)

        if self.last_s_unwrapped is None:
            # First call: initialize unwrapped s
            self.last_s_unwrapped = s_raw

        # Choose s near previous unwrapped s to avoid jump at start/finish
        k = round(self.last_s_unwrapped / L)
        candidates = [
            s_raw + (k - 1) * L,
            s_raw + k * L,
            s_raw + (k + 1) * L,
        ]
        current_s_unwrapped = min(candidates, key=lambda s: abs(s - self.last_s_unwrapped))
        self.last_s_unwrapped = current_s_unwrapped

        # Prediction horizon in unwrapped s
        horizon_s_unwrapped = current_s_unwrapped + vx * np.arange(self.mpc.N + 1) * self.mpc.dt

        # Wrap to [0, L) and shift to middle copy [L, 2L] for interpolation
        # path_s_extended = [0..L, L..2L, 2L..3L] so we query in [L, 2L]
        horizon_s = np.mod(horizon_s_unwrapped, L) + L

        # Interpolate curvature and ref_v along EXTENDED s using centered horizon
        k_profile = np.interp(horizon_s, self.path_s_extended, self.path_np_extended[:, 3])
        v_profile = np.interp(horizon_s, self.path_s_extended, self.path_np_extended[:, 2])

        # store for viz
        self.horizon_s = horizon_s


        # 4. Solve MPC
        x0 = np.array([ey, epsi, vx, vy, r, self.mpc.last_delta])

        steer_cmd, throttle_cmd, preds = self.mpc.solve(x0, v_profile, k_profile)

        # 5. Publish
        self._publish_control(throttle_cmd, steer_cmd)
        self._publish_viz(preds)

        # Timing stats
        dur = (self.get_clock().now() - t0).nanoseconds / 1e6  # ms
        msg = Float64MultiArray()
        msg.data = [dur]
        self.exec_time_pub.publish(msg)

    # --- Publishing ---

    def _publish_control(self, throttle, steer):
        # Clip
        throttle = max(-1.0, min(1.0, throttle))
        steer = max(self.mpc.delta_min, min(self.mpc.delta_max, steer))

        now = self.get_clock().now().to_msg()

        msg = ControlCommand()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.steering = steer
        # Assuming all 4 throttles share same command in [-1, 1]
        msg.throttle_fl = throttle
        msg.throttle_fr = throttle
        msg.throttle_rl = throttle
        msg.throttle_rr = throttle
        self.control_pub.publish(msg)

        if self.adapter == "pacsim":
            s = StampedScalar()
            s.stamp = now
            s.value = float(steer)
            self.debug_steer.publish(s)
            a = StampedScalar()
            a.stamp = now
            a.value = float(throttle)
            self.debug_acc.publish(a)

    def _publish_viz(self, preds):
    # Need predictions, path and horizon_s
        if (
            not preds
            or self.path_np is None
            or self.path_s is None
            or self.path_heading is None
            or self.horizon_s is None
        ):
            return

        pts = []

        for i, state in enumerate(preds):
            if i >= len(self.horizon_s):
                break

            # horizon_s is already centered in [L, 2L]
            s = float(self.horizon_s[i])

            x_ref = float(np.interp(s, self.path_s_extended, self.path_np_extended[:, 0]))
            y_ref = float(np.interp(s, self.path_s_extended, self.path_np_extended[:, 1]))
            psi_ref = float(np.interp(s, self.path_s_extended, self.path_heading_extended))

            ey = float(state[0])

            x = x_ref - ey * math.sin(psi_ref)
            y = y_ref + ey * math.cos(psi_ref)

            p = Point()
            p.x = x
            p.y = y
            pts.append(p)

        m = Marker()
        m.header.frame_id = "map"
        m.type = Marker.LINE_STRIP
        m.scale.x = 0.1
        m.color.a = 1.0
        m.color.g = 1.0
        m.points = pts

        ma = MarkerArray()
        ma.markers.append(m)
        self.viz_pub.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = MpcControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
