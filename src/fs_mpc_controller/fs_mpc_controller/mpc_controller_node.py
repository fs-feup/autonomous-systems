import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import scipy.sparse as sp
import osqp

# Messages
from custom_interfaces.msg import PathPointArray, Pose, Velocities
from pacsim.msg import StampedScalar
from geometry_msgs.msg import TwistWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray

# Global for warm-starting
_prev_solution = None

# ==================== TUNING PARAMETERS ====================
# --- Vehicle parameters ---
m = 240.0  # kg
Iz = 101.082  # kg*m^2
lf = 0.726  # m
lr = 0.804  # m
L = lf + lr  # wheelbase

# Cornering stiffness
Cf = 2.27e4  # N/rad (front axle)
Cr = 2.05e4  # N/rad (rear axle)

# --- MPC Parameters ---
N = 100          # Horizon length (timesteps)
dt = 0.02       # Timestep (seconds) - increased for efficiency

# MPC Cost Function Weights
W_EY = 100.0      # Lateral error penalty
W_EPSI = 10.0     # Heading error penalty
W_DELTA = 1.0     # Steering magnitude penalty
W_DDELTA = 50.0   # Steering rate penalty (CRITICAL)
W_SLACK = 1000.0  # Track bound slack penalty

# Input limits
DELTA_MAX = np.deg2rad(20.0)  # Max steering angle (rad)
DELTA_RATE_MAX = np.deg2rad(20.0) * dt  # Max rate per step

# Track constraints
TRACK_HALF_WIDTH = 1.0  # Soft track bounds (m)

# --- Longitudinal PID Controller Parameters ---
KP_VEL = 0.5
KI_VEL = 0.01
KD_VEL = 0.01
INTEGRAL_MAX = 2.0
THROTTLE_MAX = 1.0
THROTTLE_MIN = -1.0
AX_MAX = 5.0
AX_MIN = -5.0

# ===========================================================
# ------------------ Geometry/Path Helpers ------------------ #
def finite_diff_heading_and_curvature(xs, ys):
    """Compute heading and curvature using finite differences."""
    xs = np.asarray(xs)
    ys = np.asarray(ys)
    dx = np.gradient(xs)
    dy = np.gradient(ys)
    heading = np.arctan2(dy, dx)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = (dx*dx + dy*dy)**1.5 + 1e-9
    kappa = (dx*ddy - dy*ddx) / denom
    return heading, kappa


def wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return (a + np.pi) % (2*np.pi) - np.pi


def project_pose_to_path(x, y, xs, ys):
    """Find nearest point on path to (x,y)."""
    pts = np.vstack([xs, ys]).T
    diffs = pts - np.array([x, y])
    dists2 = np.sum(diffs*diffs, axis=1)
    idx = int(np.argmin(dists2))
    return idx


def get_path_horizon(x, y, xs, ys, vrefs, horizon_length):
    """Extract path horizon starting from current position."""
    idx = project_pose_to_path(x, y, xs, ys)
    end_idx = min(idx + horizon_length, len(xs))
    if end_idx - idx < horizon_length:
        horizon_xs = np.concatenate([xs[idx:end_idx],
                                     np.full(horizon_length - (end_idx - idx), xs[-1])])
        horizon_ys = np.concatenate([ys[idx:end_idx],
                                     np.full(horizon_length - (end_idx - idx), ys[-1])])
        horizon_vrefs = np.concatenate([vrefs[idx:end_idx],
                                        np.full(horizon_length - (end_idx - idx), vrefs[-1])])
    else:
        horizon_xs = xs[idx:end_idx]
        horizon_ys = ys[idx:end_idx]
        horizon_vrefs = vrefs[idx:end_idx]
    return horizon_xs, horizon_ys, horizon_vrefs, idx


def compute_local_errors(x, y, theta, xs, ys, psi_path):
    """Compute lateral error (ey) and heading error (epsi) in path frame."""
    idx = project_pose_to_path(x, y, xs, ys)
    px, py, psi = xs[idx], ys[idx], psi_path[idx]
    cos_psi = np.cos(psi)
    sin_psi = np.sin(psi)
    dx = x - px
    dy = y - py
    ey = -sin_psi * dx + cos_psi * dy
    epsi = wrap_angle(theta - psi)
    return ey, epsi, idx


# ------------------ Bicycle Model Dynamics ------------------ #
def build_lateral_bicycle_matrices(vx):
    """Build A, B matrices for lateral bicycle model."""
    vx_eff = max(1.0, float(vx))
    A = np.zeros((4, 4))
    B = np.zeros((4, 1))

    A[0, 1] = vx_eff
    A[0, 2] = 1.0
    A[1, 3] = 1.0

    A[2, 2] = -(Cf + Cr) / (m * vx_eff)
    A[2, 3] = -(Cf*lf - Cr*lr) / (m * vx_eff) - vx_eff
    B[2, 0] = Cf / m

    A[3, 2] = -(Cf*lf - Cr*lr) / (Iz * vx_eff)
    A[3, 3] = -(Cf*lf*lf + Cr*lr*lr) / (Iz * vx_eff)
    B[3, 0] = Cf * lf / Iz

    Ad = np.eye(4) + A * dt
    Bd = B * dt
    return Ad, Bd


# ------------------ MPC Problem Formulation ------------------ #
def build_mpc_matrices(Ad_list, Bd_list, g_list, x0, delta_prev):
    """Build QP constraint matrices."""
    nx = 4
    nu = 1
    ns = N + 1
    nz = (N+1)*nx + N*nu + ns

    def state_idx(k): return k * nx
    def input_idx(k): return (N+1) * nx + k * nu
    def slack_idx(k): return (N+1) * nx + N * nu + k

    # Equality constraints: dynamics
    rows, cols, data = [], [], []
    beq = []
    for i in range(nx):
        rows.append(i)
        cols.append(state_idx(0) + i)
        data.append(1.0)
        beq.append(x0[i])
    eq_row = nx

    for k in range(N):
        Ad = Ad_list[k]
        Bd = Bd_list[k]
        gk = g_list[k]
        for i in range(nx):
            rows.append(eq_row + i)
            cols.append(state_idx(k+1) + i)
            data.append(1.0)
            for j in range(nx):
                if Ad[i, j] != 0:
                    rows.append(eq_row + i)
                    cols.append(state_idx(k) + j)
                    data.append(-Ad[i, j])
            for j in range(nu):
                if Bd[i, j] != 0:
                    rows.append(eq_row + i)
                    cols.append(input_idx(k) + j)
                    data.append(-Bd[i, j])
            beq.append(gk[i])
        eq_row += nx

    Aeq = sp.csc_matrix((data, (rows, cols)), shape=(len(beq), nz))
    beq = np.array(beq)

    # Inequality constraints
    ineq_rows, ineq_cols, ineq_data = [], [], []
    l_ineq, u_ineq = [], []
    ineq_row = 0

    # Track bounds with slack
    for k in range(N+1):
        ey_idx = state_idx(k) + 0
        s_idx = slack_idx(k)
        # ey - s <= TRACK_HALF_WIDTH
        ineq_rows.extend([ineq_row, ineq_row])
        ineq_cols.extend([ey_idx, s_idx])
        ineq_data.extend([1.0, -1.0])
        l_ineq.append(-np.inf)
        u_ineq.append(TRACK_HALF_WIDTH)
        ineq_row += 1
        # -ey - s <= TRACK_HALF_WIDTH
        ineq_rows.extend([ineq_row, ineq_row])
        ineq_cols.extend([ey_idx, s_idx])
        ineq_data.extend([-1.0, -1.0])
        l_ineq.append(-np.inf)
        u_ineq.append(TRACK_HALF_WIDTH)
        ineq_row += 1

    # Steering bounds
    for k in range(N):
        delta_idx = input_idx(k)
        ineq_rows.append(ineq_row)
        ineq_cols.append(delta_idx)
        ineq_data.append(1.0)
        l_ineq.append(-DELTA_MAX)
        u_ineq.append(DELTA_MAX)
        ineq_row += 1

    # Steering rate
    for k in range(N):
        delta_idx = input_idx(k)
        if k == 0:
            ineq_rows.append(ineq_row)
            ineq_cols.append(delta_idx)
            ineq_data.append(1.0)
            l_ineq.append(delta_prev - DELTA_RATE_MAX)
            u_ineq.append(delta_prev + DELTA_RATE_MAX)
        else:
            delta_prev_idx = input_idx(k-1)
            ineq_rows.extend([ineq_row, ineq_row])
            ineq_cols.extend([delta_idx, delta_prev_idx])
            ineq_data.extend([1.0, -1.0])
            l_ineq.append(-DELTA_RATE_MAX)
            u_ineq.append(DELTA_RATE_MAX)
        ineq_row += 1

    # Slack >= 0
    for k in range(N+1):
        s_idx = slack_idx(k)
        ineq_rows.append(ineq_row)
        ineq_cols.append(s_idx)
        ineq_data.append(1.0)
        l_ineq.append(0.0)
        u_ineq.append(np.inf)
        ineq_row += 1

    Aineq = sp.csc_matrix((ineq_data, (ineq_rows, ineq_cols)), shape=(len(l_ineq), nz))
    l_ineq = np.array(l_ineq)
    u_ineq = np.array(u_ineq)

    A = sp.vstack([Aeq, Aineq], format='csc')
    l = np.hstack([beq, l_ineq])
    u = np.hstack([beq, u_ineq])
    return A, l, u, nz


def build_mpc_cost(vrefs, delta_prev, nz):
    """Build cost matrices P and q."""
    nx = 4
    nu = 1
    def state_idx(k): return k * nx
    def input_idx(k): return (N+1) * nx + k * nu
    def slack_idx(k): return (N+1) * nx + N * nu + k

    P_rows, P_cols, P_data = [], [], []
    q = np.zeros(nz)

    # State costs
    for k in range(N+1):
        ey_idx = state_idx(k) + 0
        epsi_idx = state_idx(k) + 1
        P_rows.extend([ey_idx, epsi_idx])
        P_cols.extend([ey_idx, epsi_idx])
        P_data.extend([2.0 * W_EY, 2.0 * W_EPSI])

    # Input costs
    for k in range(N):
        delta_idx = input_idx(k)
        P_rows.append(delta_idx)
        P_cols.append(delta_idx)
        P_data.append(2.0 * W_DELTA)

        if k == 0:
            P_rows.append(delta_idx)
            P_cols.append(delta_idx)
            P_data.append(2.0 * W_DDELTA)
            q[delta_idx] += -2.0 * W_DDELTA * delta_prev
        else:
            delta_prev_idx = input_idx(k-1)
            P_rows.extend([delta_idx, delta_idx, delta_prev_idx, delta_prev_idx])
            P_cols.extend([delta_idx, delta_prev_idx, delta_idx, delta_prev_idx])
            P_data.extend([2.0*W_DDELTA, -2.0*W_DDELTA, -2.0*W_DDELTA, 2.0*W_DDELTA])

    # Slack costs
    for k in range(N+1):
        s_idx = slack_idx(k)
        P_rows.append(s_idx)
        P_cols.append(s_idx)
        P_data.append(2.0 * W_SLACK)

    P = sp.csc_matrix((P_data, (P_rows, P_cols)), shape=(nz, nz))
    P = (P + P.T) / 2.0
    P = P + sp.eye(nz) * 1e-4  # Increased regularization
    return P, q


# ------------------ MPC Solver ------------------ #
def mpc_solve_lateral(current_pose, current_vx, delta_prev, path_points, logger=None):
    """Solve lateral MPC and return steering + predicted states."""
    x, y, theta = current_pose
    path_points = np.asarray(path_points)
    xs_full = path_points[:, 0]
    ys_full = path_points[:, 1]
    vrefs_full = path_points[:, 2]

    xs, ys, vrefs, start_idx = get_path_horizon(x, y, xs_full, ys_full, vrefs_full, N+1)

    if logger:
        logger.info(f"[MPC] Pose: ({x:.2f}, {y:.2f}, {np.rad2deg(theta):.1f}deg), vx={current_vx:.2f}m/s")
        logger.info(f"[MPC] Path start_idx={start_idx}, vref=[{vrefs.min():.2f}, {vrefs.max():.2f}]m/s")

    # Low speed fallback
    if current_vx < 0.5:
        if logger:
            logger.warn(f"[MPC] Low speed, using P controller")
        psi_path, _ = finite_diff_heading_and_curvature(xs, ys)
        target_heading = psi_path[min(3, len(psi_path)-1)]
        heading_error = wrap_angle(target_heading - theta)
        delta_cmd = float(np.clip(1.5 * heading_error, -DELTA_MAX, DELTA_MAX))
        dummy_states = np.zeros((N+1, 4))
        dummy_states[:, 0] = np.linspace(0, 0.1, N+1)
        dummy_states[:, 1] = heading_error
        return delta_cmd, dummy_states

    # Compute path properties
    psi_path, kappas = finite_diff_heading_and_curvature(xs, ys)
    ey0, epsi0, _ = compute_local_errors(x, y, theta, xs, ys, psi_path)

    if logger:
        logger.info(f"[MPC] Errors: ey={ey0:.3f}m, epsi={np.rad2deg(epsi0):.1f}deg")

    x0 = np.array([ey0, epsi0, 0.0, 0.0])

    # Build dynamics
    Ad_list, Bd_list, g_list = [], [], []
    for k in range(N):
        v_scheduled = max(1.0, float(vrefs[min(k, len(vrefs)-1)]))
        Ad, Bd = build_lateral_bicycle_matrices(v_scheduled)
        Ad_list.append(Ad)
        Bd_list.append(Bd)
        gk = np.zeros(4)
        kappa_k = float(kappas[min(k, len(kappas)-1)])
        gk[1] = -dt * v_scheduled * kappa_k
        g_list.append(gk)

    # Build QP
    A, l, u, nz = build_mpc_matrices(Ad_list, Bd_list, g_list, x0, delta_prev)
    P, q = build_mpc_cost(vrefs, delta_prev, nz)

    if logger:
        logger.info(f"[MPC] QP size: nz={nz}, constraints={A.shape[0]}")

    # Solve
    try:
        prob = osqp.OSQP()
        prob.setup(P=P, q=q, A=A, l=l, u=u,
                   verbose=False, polish=False,
                   eps_abs=1e-3, eps_rel=1e-3, max_iter=200)

        global _prev_solution
        if _prev_solution is not None and len(_prev_solution) == nz:
            prob.warm_start(x=_prev_solution)

        res = prob.solve()
    except Exception as e:
        if logger:
            logger.error(f"[MPC] OSQP setup failed: {e}")
        delta_cmd = float(np.clip(delta_prev, -DELTA_MAX, DELTA_MAX))
        dummy_states = np.zeros((N+1, 4))
        dummy_states[0, 0] = ey0
        dummy_states[0, 1] = epsi0
        return delta_cmd, dummy_states

    if logger:
        logger.info(f"[MPC] Status: {res.info.status} ({res.info.iter} iters)")

    # Failure fallback
    if res.info.status_val not in (1, 2) or res.x is None:
        if logger:
            logger.warn(f"[MPC] Solve failed, using emergency steering")
        target_heading = psi_path[min(5, len(psi_path)-1)]
        heading_error = wrap_angle(target_heading - theta)
        delta_cmd = float(np.clip(2.0 * heading_error, -DELTA_MAX, DELTA_MAX))
        dummy_states = np.zeros((N+1, 4))
        dummy_states[:, 0] = np.linspace(ey0, 0.0, N+1)
        dummy_states[:, 1] = np.linspace(heading_error, 0.0, N+1)
        return delta_cmd, dummy_states

    # Success
    delta_idx = (N+1) * 4
    delta_cmd = float(np.clip(res.x[delta_idx], -DELTA_MAX, DELTA_MAX))

    if logger:
        logger.info(f"[MPC] Steering: {np.rad2deg(delta_cmd):.2f}deg")

    predicted_states = np.array([res.x[k*4:(k+1)*4] for k in range(N+1)])

    # Warm-start for next cycle
    _prev_solution = res.x.copy()

    return delta_cmd, predicted_states


# ------------------ Longitudinal PID Controller ------------------ #
class VelocityPIDController:
    def __init__(self):
        self.kp = KP_VEL
        self.ki = KI_VEL
        self.kd = KD_VEL
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, v_current, v_target, current_time, logger=None):
        error = v_target - v_current
        if self.prev_time is None:
            dt_control = dt
        else:
            dt_control = max(current_time - self.prev_time, 1e-6)

        p_term = self.kp * error
        self.integral += error * dt_control
        self.integral = np.clip(self.integral, -INTEGRAL_MAX, INTEGRAL_MAX)
        i_term = self.ki * self.integral

        d_term = 0.0
        if self.prev_time is not None:
            d_term = self.kd * (error - self.prev_error) / dt_control

        throttle = np.clip(p_term + i_term + d_term, THROTTLE_MIN, THROTTLE_MAX)

        self.prev_error = error
        self.prev_time = current_time

        if logger:
            logger.info(f"[PID] v_err={error:.2f}, P={p_term:.3f}, I={i_term:.3f}, D={d_term:.3f}, throttle={throttle:.3f}")
        return float(throttle)


# ------------------ ROS2 Node ------------------ #
class LpvMpcControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')
        self.velocity_pid = VelocityPIDController()

        # Subscribers
        self.path_sub = self.create_subscription(PathPointArray, '/path_planning/path', self.path_callback, 10)
        self.pose_sub = self.create_subscription(Pose, '/state_estimation/vehicle_pose', self.pose_callback, 10)
        self.velocity_sub = self.create_subscription(TwistWithCovarianceStamped, '/pacsim/velocity', self.velocity_callback, 10)

        # Publishers
        self.steering_pub = self.create_publisher(StampedScalar, '/pacsim/steering_setpoint', 10)
        self.throttle_pub = self.create_publisher(StampedScalar, '/pacsim/throttle_setpoint', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mpc/predicted_trajectory', 10)

        # Timer
        self.control_timer = self.create_timer(dt, self.update_control)

        # State
        self.path_points = []
        self.current_pose = (0.0, 0.0, 0.0)
        self.current_velocity = 0.0
        self.previous_steering = 0.0
        self.control_count = 0

        self.get_logger().info("=== MPC + PID Controller Initialized ===")
        self.get_logger().info(f"MPC: N={N}, dt={dt}, W_DDELTA={W_DDELTA}")
        self.get_logger().info(f"PID: Kp={KP_VEL}, Ki={KI_VEL}, Kd={KD_VEL}")

    def path_callback(self, msg):
        self.path_points = [(p.x, p.y, p.v) for p in msg.pathpoint_array]

    def pose_callback(self, msg):
        self.current_pose = (msg.x, msg.y, msg.theta)

    def velocity_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x

    def update_control(self):
        self.control_count += 1
        if len(self.path_points) < N + 1:
            return

        try:
            log_this = (self.control_count % 25 == 0)
            logger = self.get_logger() if log_this else None

            current_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                           self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

            # Lateral MPC
            steering_cmd, predicted_states = mpc_solve_lateral(
                self.current_pose,
                self.current_velocity,
                self.previous_steering,
                self.path_points,
                logger=logger
            )

            if log_this:
                path_array = np.asarray(self.path_points)
                xs = path_array[:, 0]
                ys = path_array[:, 1]
                psi_path, _ = finite_diff_heading_and_curvature(xs, ys)
                idx = project_pose_to_path(self.current_pose[0], self.current_pose[1], xs, ys)
                self.publish_predicted_trajectory(predicted_states, psi_path[idx])

            # Longitudinal PID
            path_array = np.asarray(self.path_points)
            xs = path_array[:, 0]
            ys = path_array[:, 1]
            vrefs = path_array[:, 2]
            idx = project_pose_to_path(self.current_pose[0], self.current_pose[1], xs, ys)
            v_target = float(vrefs[idx])
            throttle_cmd = self.velocity_pid.compute(
                self.current_velocity, v_target, current_time, logger=logger
            )

            # Publish
            timestamp = self.get_clock().now().to_msg()
            steering_msg = StampedScalar()
            steering_msg.stamp = timestamp
            steering_msg.value = steering_cmd
            self.steering_pub.publish(steering_msg)

            throttle_msg = StampedScalar()
            throttle_msg.stamp = timestamp
            throttle_msg.value = throttle_cmd
            self.throttle_pub.publish(throttle_msg)

            self.previous_steering = steering_cmd

            if log_this:
                self.get_logger().info(f"[CONTROL #{self.control_count}] Steering={np.rad2deg(steering_cmd):.2f}deg, Throttle={throttle_cmd:.3f}")

        except Exception as e:
            self.get_logger().error(f"Control failed: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def publish_predicted_trajectory(self, predicted_states, path_heading):
        marker_array = MarkerArray()
        x0, y0, theta0 = self.current_pose

        delete_marker = Marker()
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "mpc_prediction"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        trajectory_marker = Marker()
        trajectory_marker.header.frame_id = "map"
        trajectory_marker.header.stamp = self.get_clock().now().to_msg()
        trajectory_marker.ns = "mpc_prediction"
        trajectory_marker.id = 0
        trajectory_marker.type = Marker.LINE_STRIP
        trajectory_marker.action = Marker.ADD
        trajectory_marker.scale.x = 0.05
        trajectory_marker.color.r = 0.0
        trajectory_marker.color.g = 1.0
        trajectory_marker.color.b = 0.0
        trajectory_marker.color.a = 0.8

        from geometry_msgs.msg import Point
        for k in range(len(predicted_states)):
            ey = predicted_states[k, 0]
            x_pred = x0 + k * dt * self.current_velocity * np.cos(theta0) - ey * np.sin(theta0)
            y_pred = y0 + k * dt * self.current_velocity * np.sin(theta0) + ey * np.cos(theta0)
            point = Point()
            point.x = x_pred
            point.y = y_pred
            point.z = 0.1
            trajectory_marker.points.append(point)
        marker_array.markers.append(trajectory_marker)

        for k in range(0, len(predicted_states), 3):
            ey = predicted_states[k, 0]
            x_pred = x0 + k * dt * self.current_velocity * np.cos(theta0) - ey * np.sin(theta0)
            y_pred = y0 + k * dt * self.current_velocity * np.sin(theta0) + ey * np.cos(theta0)
            point_marker = Marker()
            point_marker.header.frame_id = "map"
            point_marker.header.stamp = self.get_clock().now().to_msg()
            point_marker.ns = "mpc_prediction"
            point_marker.id = k + 1
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = x_pred
            point_marker.pose.position.y = y_pred
            point_marker.pose.position.z = 0.1
            point_marker.scale.x = point_marker.scale.y = point_marker.scale.z = 0.1
            point_marker.color.r = 0.0
            point_marker.color.g = 0.8
            point_marker.color.b = 1.0
            point_marker.color.a = 0.6
            marker_array.markers.append(point_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LpvMpcControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()