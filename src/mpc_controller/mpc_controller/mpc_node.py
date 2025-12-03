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
    """Optimized PI-D controller."""
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
    """Acados-based Dynamic Bicycle MPC with Aero and Pacejka."""

    def __init__(self, car_params: dict, control_params: dict):
        self.model_name = "fs_dynamic_bicycle"

        # Car Parameters
        self.m = float(car_params.get("mass", 150.0))
        self.Iz = float(car_params.get("Izz", 80.0))

        wb = float(car_params.get("wheel_base", 1.53))
        dist_cg_r = float(car_params.get("dist_cg_2_rear_axis", 0.8))
        self.Lr = dist_cg_r
        self.Lf = wb - self.Lr

        # Aero
        self.Cd = float(car_params.get("drag_coefficient", 0.7))
        self.Cl = float(car_params.get("lift_coefficient", -1.0))  # Negative for downforce
        self.A_front = float(car_params.get("frontal_area", 1.0))
        self.rho = 1.225

        # Pacejka (Magnitude coefficients, signs handled in equations)
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

        # Weights
        self.Q_diag = [
            control_params.get("mpc_w_ey", 80.0),
            control_params.get("mpc_w_epsi", 30.0),
            control_params.get("mpc_w_vy", 5.0),
            control_params.get("mpc_w_r", 2.0),
            control_params.get("mpc_w_delta", 0.5),
        ]
        self.R_diag = [control_params.get("mpc_w_d_delta", 1.0)]

        self.last_u = 0.0
        self._setup_acados()

    def _setup_acados(self):
        # 1. Define Model
        model = AcadosModel()
        model.name = self.model_name

        # States: [lateral_error, heading_error, lateral_velocity, yaw_rate, steering_angle]
        ey = SX.sym("ey")
        epsi = SX.sym("epsi")
        vy = SX.sym("vy")
        r = SX.sym("r")
        delta = SX.sym("delta")
        x = vertcat(ey, epsi, vy, r, delta)

        # Controls: [steering_rate]
        ddelta = SX.sym("ddelta")
        u = vertcat(ddelta)

        # Parameters: [vx, curvature]
        p_vx = SX.sym("p_vx")
        p_k = SX.sym("p_k")
        p = vertcat(p_vx, p_k)

        # Dynamics
        vx_safe = if_else(p_vx < 1.0, 1.0, p_vx)

        alpha_f = delta - atan((vy + self.Lf * r) / vx_safe)
        alpha_r = -atan((vy - self.Lr * r) / vx_safe)

        # Vertical Loads (Simplified Aero)
        Fz_static = self.m * 9.81
        Fz_aero = 0.5 * self.rho * self.Cl * self.A_front * p_vx * p_vx
        Fz_tot = Fz_static - Fz_aero
        Fz_f = (self.Lr / (self.Lf + self.Lr)) * Fz_tot
        Fz_r = (self.Lf / (self.Lf + self.Lr)) * Fz_tot

        # Pacejka Forces
        Fy_f = Fz_f * self.D_tire * sin(self.C_tire * atan(self.B_tire * alpha_f))
        Fy_r = Fz_r * self.D_tire * sin(self.C_tire * atan(self.B_tire * alpha_r))

        # State Derivatives (Curvilinear coordinates)
        dey = vy * cos(epsi) + p_vx * sin(epsi)
        depsi = r - p_k * p_vx
        dvy = (Fy_f * cos(delta) + Fy_r) / self.m - p_vx * r
        dr = (self.Lf * Fy_f * cos(delta) - self.Lr * Fy_r) / self.Iz
        ddelta_dt = ddelta

        model.f_expl_expr = vertcat(dey, depsi, dvy, dr, ddelta_dt)
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

        # Cost Function (LSQ)
        ny = 6  # states(5) + control(1)
        ny_e = 5  # terminal: states only

        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"

        ocp.cost.W = np.diag(self.Q_diag + self.R_diag)
        ocp.cost.W_e = np.diag(self.Q_diag)

        ocp.cost.Vx = np.zeros((ny, 5))
        ocp.cost.Vx[:5, :5] = np.eye(5)
        ocp.cost.Vu = np.zeros((ny, 1))
        ocp.cost.Vu[5, 0] = 1.0

        ocp.cost.Vx_e = np.eye(5)

        ocp.cost.yref = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(ny_e)

        # Initial state constraint (all states fixed at stage 0)
        ocp.constraints.x0 = np.zeros(5)

        # Path Constraints on ey and delta
        ocp.constraints.lbx = np.array([-1.0, self.delta_min])
        ocp.constraints.ubx = np.array([1.0, self.delta_max])
        ocp.constraints.idxbx = np.array([0, 4])  # ey, delta

        # Control bounds
        ocp.constraints.lbu = np.array([-self.delta_rate_max])
        ocp.constraints.ubu = np.array([self.delta_rate_max])
        ocp.constraints.idxbu = np.array([0])

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
        # x0: [ey, epsi, vy, r, delta]

        # Set Initial State as bounds at stage 0
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)

        # Set parameters and stage references
        for i in range(self.N):
            vx = vx_ref_profile[i]
            k = k_ref_profile[i]

            # Param: [vx, k]
            self.solver.set(i, "p", np.array([vx, k]))

            # Reference: yaw rate matches curvature, rest zero
            r_ref = vx * k
            yref = np.array([0.0, 0.0, 0.0, r_ref, 0.0, 0.0])
            self.solver.set(i, "yref", yref)

        # Terminal stage
        vx_N = vx_ref_profile[self.N]
        k_N = k_ref_profile[self.N]
        self.solver.set(self.N, "p", np.array([vx_N, k_N]))
        yref_e = np.array([0.0, 0.0, 0.0, vx_N * k_N, 0.0])

        # *** FIX: use "yref" at stage N instead of invalid "yref_e" ***
        self.solver.set(self.N, "yref", yref_e)

        # Solve
        status = self.solver.solve()

        if status != 0:
            print(f"[MPC ERROR] Acados solver failed. Status: {status}")
            return self.last_u, 0.0, []

        # Get Control
        u0 = self.solver.get(0, "u")
        d_delta = u0[0]

        self.last_u = x0[4] + d_delta * self.dt

        # Get Predictions for viz
        preds = []
        for i in range(self.N + 1):
            preds.append(self.solver.get(i, "x"))

        return self.last_u, d_delta, preds


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
        self.pid = LongitudinalPID(control_cfg)
        self.mpc = LateralMPC(car_cfg, control_cfg)

        # State
        self.path_np = None
        self.path_s = None
        self.current_pose = None
        self.current_vel = None
        self.go_signal = True if self.adapter == "pacsim" else False
        self.path_np = None
        self.path_s = None
        self.path_heading = None
        self.horizon_s = None

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
        self.time_pub = self.create_publisher(Float64MultiArray, "/control/exec_time", 1)

        # Timer
        self.timer_period = float(control_cfg.get("command_time_interval", 20)) / 1000.0
        self.timer = self.create_timer(self.timer_period, self._control_loop)

        self.get_logger().info(f"Optimization MPC Ready. Adapter: {self.adapter}")

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
        # Convert path to efficient NumPy arrays ONCE
        if not msg.pathpoint_array:
            return

        n = len(msg.pathpoint_array)
        data = np.zeros((n, 4))  # x, y, v, kappa

        for i, p in enumerate(msg.pathpoint_array):
            data[i, 0] = p.x
            data[i, 1] = p.y
            data[i, 2] = p.v
            data[i, 3] = 0.0  # Placeholder for curvature

        # Compute curvature using 3-point circle fit
        if n > 2:
            p_m = data[:-2, :2]
            p = data[1:-1, :2]
            p_p = data[2:, :2]

            a = np.linalg.norm(p - p_m, axis=1)
            b = np.linalg.norm(p_p - p, axis=1)
            c = np.linalg.norm(p_p - p_m, axis=1)

            s_tri = (a + b + c) / 2
            area = np.sqrt(np.abs(s_tri * (s_tri - a) * (s_tri - b) * (s_tri - c)))

            kappa = 4 * area / (a * b * c + 1e-6)

            v1 = p - p_m
            v2 = p_p - p
            cross = v1[:, 0] * v2[:, 1] - v1[:, 1] * v2[:, 0]
            kappa = np.where(cross < 0, -kappa, kappa)

            data[1:-1, 3] = kappa
            data[0, 3] = kappa[0]
            data[-1, 3] = kappa[-1]

        # Cumulative distance for interpolation
        deltas = np.diff(data[:, :2], axis=0)
        dists = np.sqrt((deltas ** 2).sum(axis=1))
        s = np.concatenate(([0], np.cumsum(dists)))

        self.path_np = data
        self.path_s = s
        
        n = data.shape[0]
        headings = np.zeros(n)
        headings[:-1] = np.arctan2(deltas[:, 1], deltas[:, 0])
        headings[-1] = headings[-2]  # repeat last heading
        self.path_heading = headings

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
        idx_min = np.argmin(dists_sq)

        closest_x = self.path_np[idx_min, 0]
        closest_y = self.path_np[idx_min, 1]

        idx_next = min(idx_min + 1, len(self.path_np) - 1)
        path_dx = self.path_np[idx_next, 0] - closest_x
        path_dy = self.path_np[idx_next, 1] - closest_y
        path_heading = math.atan2(path_dy, path_dx)

        ey = -math.sin(path_heading) * (px - closest_x) + math.cos(path_heading) * (py - closest_y)
        epsi = _wrap_angle(theta - path_heading)

        ref_v = self.path_np[idx_min, 2]

        # 3. Reference Generation (Spline/Lookup)
        current_s = self.path_s[idx_min]
        horizon_s = current_s + vx * np.arange(self.mpc.N + 1) * self.mpc.dt

        # Interpolate curvature and ref_v along s
        horizon_s = np.clip(horizon_s, 0, self.path_s[-1])

        k_profile = np.interp(horizon_s, self.path_s, self.path_np[:, 3])
        v_profile = np.interp(horizon_s, self.path_s, self.path_np[:, 2])

        # store for viz
        self.horizon_s = horizon_s

        # 4. Solve MPC
        x0 = np.array([ey, epsi, vy, r, self.mpc.last_u])

        next_u, d_delta, preds = self.mpc.solve(x0, v_profile, k_profile)

        # 5. Longitudinal Control
        accel = self.pid.update(ref_v, vx)

        # 6. Publish
        self._publish_control(accel, next_u)
        self._publish_viz(preds)

        # Timing stats
        dur = (self.get_clock().now() - t0).nanoseconds / 1e6
        msg = Float64MultiArray()
        msg.data = [dur]
        self.time_pub.publish(msg)

    # --- Publishing ---

    def _publish_control(self, acc, steer):
        acc = max(-1.0, min(1.0, acc))
        steer = max(self.mpc.delta_min, min(self.mpc.delta_max, steer))

        now = self.get_clock().now().to_msg()

        msg = ControlCommand()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.steering = steer
        msg.throttle_fl = acc
        msg.throttle_fr = acc
        msg.throttle_rl = acc
        msg.throttle_rr = acc
        self.control_pub.publish(msg)

        if self.adapter == "pacsim":
            s = StampedScalar()
            s.stamp = now
            s.value = float(steer)
            self.debug_steer.publish(s)
            a = StampedScalar()
            a.stamp = now
            a.value = float(acc)
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
            # Clamp index in case of off-by-one
            if i >= len(self.horizon_s):
                break

            s = self.horizon_s[i]
            # Ensure s inside known path range
            s = max(self.path_s[0], min(self.path_s[-1], s))

            # Reference centerline position and heading at arc-length s
            x_ref = float(np.interp(s, self.path_s, self.path_np[:, 0]))
            y_ref = float(np.interp(s, self.path_s, self.path_np[:, 1]))
            psi_ref = float(np.interp(s, self.path_s, self.path_heading))

            ey = float(state[0])   # lateral error from MPC state

            # Convert ey (track-normal offset) to global position
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
