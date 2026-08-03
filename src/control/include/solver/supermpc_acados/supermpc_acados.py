import sys
import os
import shutil
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat, sin, cos, sqrt, atan, atan2, tan, if_else, fabs, tanh, fmin, fabs
from ament_index_python.packages import get_package_prefix
import yaml

"""SuperMPC - a path-tracking NMPC whose SPEED is decided by the car's dynamics.

Derived from bombated_mpc: identical (validated) 16-state four-wheel plant, but a
different objective.

bombated_mpc tracks the planner's velocity profile with a two-sided quadratic
term, which makes the profile an ORDER. If the planner asks for more speed than
the tyres can deliver, the optimizer faithfully obeys and the car leaves the
track. SuperMPC instead:

  * splits the position error into CONTOURING (perpendicular to the path) and
    LAG (along the path). Contouring is weighted heavily - this is a path
    tracker. Lag is weighted lightly, which is what frees the controller to sit
    ahead of or behind the planner's timing and therefore to choose its own
    speed.
  * replaces velocity TRACKING with an ambition term (a mild pull towards
    v_ref * stretch) and a one-sided ceiling (a heavy penalty only above
    v_ref * cap). Below the ceiling the planner's profile exerts no downward
    force at all.
  * bounds the rear slip angles, so trajectories that require driving the rear
    axle past its grip peak are infeasible rather than merely expensive.

The net effect: the planner sets the route and an upper speed bound; the
nonlinear model decides how fast that route can actually be taken. Asking the
planner for an impossible speed makes the car run at its own limit instead of
exploding.
"""

path_size = 31
path_point_size = 4  # x, y, velocity, orientation

# Per-stage acados parameters: the path point plus the two speed thresholds.
# The thresholds are parameters (not baked-in constants) so the ambition and
# ceiling factors stay tunable from YAML without regenerating the solver.
N_PARAMS = 6
(P_X, P_Y, P_V, P_YAW, P_V_STRETCH, P_V_CAP) = range(N_PARAMS)

# ---------------------------------------------------------------------------
# Vehicle parameters.
#
# Every constant below mirrors the InvictaSim FSFEUP02 vehicle model so that the
# NMPC's internal plant matches the simulator it drives. Sources:
#   config/car/02.yaml                              (mass, inertia, geometry)
#   config/car/aero_model/02_aero.yaml              (aero)
#   config/car/load_transfer_model/02_load_transfer.yaml
#   config/car/steering_model/02_steering.yaml
#   config/car/steering_motor_model/02_steering_motor.yaml
#   config/car/motor_model/02_motor.yaml
#   config/car/transmission_model/02_transmission.yaml
#   config/car/tire_model/02_fitted_tire.yaml       (via a fit, see tyre section)
# ---------------------------------------------------------------------------

mass = 238.0            # car.total_mass
sprung_mass = 188.0     # car.sprung_mass
unsprung_mass = 50.0    # car.unsprung_mass
Izz = 101.82            # car.Izz

# FSFEUP02 uses lr = cg_2_rear_axis and lf = wheelbase - lr for the force moment
# arms and the static load split.
lr = 0.706
lf = 1.53 - lr          # 0.824
L = lr + lf

# The simulator computes the contact-patch velocities - and therefore the slip
# angles - from tire_parameters.d_fleft / d_bleft.  Those used to hold 0.99 / 1.06,
# an effective 2.05 m wheelbase against the chassis' 1.53 m, and this model had to
# mirror the inconsistency to predict the same forces.  The tyre config now carries
# the chassis geometry (0.824 / 0.706), so the slip distances are simply lf and lr.
lf_slip = lf           # tire.d_fleft / d_fright
lr_slip = lr           # tire.d_bleft / d_bright
sf = 1.2                # car.track_width (front)
sr = 1.2                # car.track_width (rear)
s = sf / 2

# AckermanSteering with ackerman_factor = 0.0 steers both front wheels to the
# same angle (pure parallel steering), so there is no Ackermann spread here.
ackermann_factor = 0.0

wheel_radius = 0.20574  # tire.effective_tire_r

# The simulator uses 0.2 / 0.4 kg m^2. With the real values the wheel-spin mode
# settles in well under a millisecond, roughly two orders of magnitude faster
# than the MPC's discretisation step, which wrecks the QP's conditioning. Since
# the mode is quasi-static on the MPC's timescale either way, inflating the
# inertia barely moves the predicted trajectory but makes the problem tractable.
front_wheel_inertia = 0.324883
rear_wheel_inertia = 0.879636
front_bearing_drag = 0

# --- Tyre -------------------------------------------------------------------
# Simple Pacejka magic formula fitted to the forces the simulator's MF6.2 model
# actually produces over a lap (recorded /invictasim/vehicle_model/tire/*):
#   Fy/Fz = -D*sin(C*atan(B*a - E*(B*a - atan(B*a))))    rms(mu) = 0.10
#   Fx/Fz =  D*sin(C*atan(B*k - E*(B*k - atan(B*k))))    rms(mu) = 0.06
tire_lateral_B = 20.19
tire_lateral_C = 1.148
tire_lateral_D = 2.129
tire_lateral_E = 0.543

tire_B = 17.58
tire_C = 1.860
tire_D = 1.747
tire_E = 1.013

# MF6.2 rolling resistance for the fitted tyre is very small (QSY1 = 0.0033).
rolling_resistance_coefficient = 0.0033

# --- Aerodynamics -----------------------------------------------------------
air_density = 1.225
drag_coefficient = 0.73
lift_coefficient = 0.86  # DefaultAeroModel: Fz = -0.5*rho*A*Cl*vx^2 (downforce)
frontal_area = 0.44
aero_balance_front = 0.5

# --- Load transfer ----------------------------------------------------------
# VDLoadTransferModel is linear in the accelerations, so its behaviour collapses
# into three constants (N per m/s^2). Derived directly from its formulas with
# the 02 parameters (sprung/unsprung split, roll centres, pitch centre):
#   longitudinal total = (m_u*h_u + m_s*h_s) / wheelbase        * ax
#   lateral front      = (m_u*mdF*h_u + m_s*mdF*h_rcf + m_s*h_ra*Kf) / track * ay
#   lateral rear       = (m_u*mdR*h_u + m_s*mdR*h_rcr + m_s*h_ra*(1-Kf)) / track * ay
unsprung_cg_z = 0.206
sprung_cg_z = 0.245
roll_axis_z = 0.05
front_roll_center_z = 0.04
rear_roll_center_z = 0.05
front_stiffness_distribution = 0.5

front_mass_distribution = lr / L
longitudinal_transfer_gain = (unsprung_mass * unsprung_cg_z + sprung_mass * sprung_cg_z) / L
lateral_transfer_gain_front = (
    unsprung_mass * front_mass_distribution * unsprung_cg_z
    + sprung_mass * front_mass_distribution * front_roll_center_z
    + sprung_mass * roll_axis_z * front_stiffness_distribution
) / sf
lateral_transfer_gain_rear = (
    unsprung_mass * (1.0 - front_mass_distribution) * unsprung_cg_z
    + sprung_mass * (1.0 - front_mass_distribution) * rear_roll_center_z
    + sprung_mass * roll_axis_z * (1.0 - front_stiffness_distribution)
) / sr

# FSFEUP02 feeds the load transfer model with the BODY-FRAME velocity
# derivatives (ds(VX), ds(VY)) passed through a first-order filter with unit
# time constant, not with the inertial accelerations. Reproduce that exactly.
# Suspension roll/pitch response, matching FSFEUP02Model::kAccelerationFilterTau.
# This was 1.0 s in both the simulator and here, which lagged the load transfer
# a full second behind the car.
acceleration_filter_tau = 0.10

# --- Powertrain -------------------------------------------------------------
gear_ratio = 3.67
transmission_efficiency = 0.900263
max_motor_torque = 145        # motor.max_peak_torque
max_motor_power = 124000.0      # motor.max_peak_power
viscous_drag_coeff = 0.044375
coulomb_drag = 0.0952027
coulomb_smooth_stiffness = 5.78895
diff_kv = 41.6054
diff_preload = 0
diff_drive_ramp = 1.42304

# DelayedInverter applies a transport delay of |throttle| * 500 ms to the torque
# request. A pure delay cannot be expressed in an ODE, so it is approximated by
# a first-order lag on the applied throttle with a representative time constant.
inverter_tau = 0.2

gravity_acceleration = 9.81

# --- Steering ---------------------------------------------------------------
steering_motor_tau = 0.150      # steering_motor.time_constant (identified from bag)
max_steering_angle = 0.335      # steering.maximum_steering_angle
max_steering_rate = 5.0         # rad/s, rate limit on the commanded angle
# Velocity floor: the model must never predict driving in reverse (see the
# state-bound section for why).
min_longitudinal_velocity = 0.0

# Rear slip angle envelope (rad). The fitted tyre peaks around 0.12-0.15 rad, so
# this sits just past the peak: usable grip stays reachable, the unstable
# far side does not.
max_rear_slip_angle = 0.16
envelope_penalty = 1e3
max_throttle_rate = 4.0         # 1/s, rate limit on the commanded throttle

# Regularisation of the wheel longitudinal velocities. The simulator uses TWO
# different values and so must this model: TireModel regularises the slip angle
# with longitudinal_epsilon = 0.5 and floors the slip-ratio denominator at 1.0.
#
# Using a single value for both is not harmless. The same term also supplies the
# direction factor Vcx/|Vcx| that flips the slip angle in reverse, and it
# attenuates the slip angle at low speed - so an oversized epsilon quietly makes
# the car understeer in the model exactly where it is slowest and most delicate.
eps_vx_angle = 0.5   # matches TireModel's longitudinal_epsilon
eps_vx_ratio = 1.0   # matches TireModel's V_floor for the slip ratio

# State layout
NX = 16
NU = 2
(X_POS, Y_POS, YAW, VX, VY, YAW_RATE, AX, AY, STEER, W_FL, W_FR, W_RL, W_RR,
 THROTTLE_CMD, STEER_CMD, THROTTLE_APPLIED) = range(NX)


def smooth_min(a, b, eps=1.0):
    """Smooth approximation of min(a, b) with continuous derivatives."""
    return 0.5 * (a + b - sqrt((a - b) ** 2 + eps ** 2))


def smooth_sign(v, stiffness):
    return (2.0 / np.pi) * atan(stiffness * v)


def smooth_abs(v, eps=1e-2):
    """|v| without the kink at zero, which would otherwise put a discontinuous
    Jacobian in the middle of the operating range (the axle torque and the motor
    speed both cross zero constantly while coasting)."""
    return sqrt(v * v + eps * eps)


def smooth_relu(z, eps=0.2):
    """max(0, z) with a continuous derivative, for one-sided penalties."""
    return 0.5 * (z + sqrt(z * z + eps * eps))


def pacejka(B, C, D, E, slip):
    return D * sin(C * atan(B * slip - E * (B * slip - atan(B * slip))))


def get_config_yaml_path(package_name: str, dir: str, filename: str) -> str:
    """
    Constructs the path to a YAML config file within the workspace.

    Args:
        package_name: Name of the ROS package
        dir: Subdirectory within the config folder
        filename: Config filename (without .yaml extension)

    Returns:
        Full path to the config YAML file
    """
    try:
        package_prefix = get_package_prefix(package_name)
    except ImportError:
        # Fallback if ament_index_python is not available
        package_prefix = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    workspace_path = os.path.join(package_prefix, "../../config", dir, f"{filename}.yaml")
    return workspace_path


def get_active_adapter() -> str:
    """
    Read the active adapter from the global config so the generated solver is
    dimensioned from the same control config the node loads at runtime.
    """
    with open(get_config_yaml_path("common_lib", "global", "global_config"), 'r') as f:
        return yaml.safe_load(f)["global"]["adapter"]


def load_mpc_parameters():
    """
    Load MPC horizon time, steps and the wheel speed scaling from the active
    adapter's control config.
    """
    with open(get_config_yaml_path("common_lib", "control", get_active_adapter()), 'r') as f:
        control_config = yaml.safe_load(f)["control"]

    mpc_horizon_time = control_config.get("supermpc_prediction_horizon_seconds",
                                          control_config["mpc_prediction_horizon_seconds"])
    mpc_horizon_steps = control_config.get("supermpc_prediction_horizon_steps",
                                           control_config["mpc_prediction_horizon_steps"])
    wheel_speeds_scale = control_config["wheel_speeds_scale_mpc"]

    return mpc_horizon_time, mpc_horizon_steps, wheel_speeds_scale


def export_mpc_model() -> AcadosModel:
    model = AcadosModel()
    model.name = "supermpc"
    model.p = SX.sym("p", N_PARAMS)

    _, _, wheel_speed_scale = load_mpc_parameters()

    x = SX.sym("x", NX)
    xdot = SX.sym("xdot", NX)
    u = SX.sym("u", NU)

    vx = x[VX]
    vy = x[VY]
    yaw_rate = x[YAW_RATE]
    steering = x[STEER]

    # --- Steering geometry --------------------------------------------------
    # ackerman_factor is 0 in the simulator, so both front wheels take the same
    # angle. The expression keeps the Ackermann term so a non-zero factor can be
    # configured later without changing the structure.
    sa_fl = atan(L * tan(steering) / (L - s * ackermann_factor * tan(steering)))
    sa_fr = atan(L * tan(steering) / (L + s * ackermann_factor * tan(steering)))
    sa_rl = 0
    sa_rr = 0

    # --- Corner velocities --------------------------------------------------
    v_fl_x = vx - yaw_rate * sf / 2
    v_fr_x = vx + yaw_rate * sf / 2
    v_rl_x = vx - yaw_rate * sr / 2
    v_rr_x = vx + yaw_rate * sr / 2

    v_fl_y = vy + yaw_rate * lf_slip
    v_fr_y = vy + yaw_rate * lf_slip
    v_rl_y = vy - yaw_rate * lr_slip
    v_rr_y = vy - yaw_rate * lr_slip

    # Contact-patch velocities resolved into each wheel's OWN frame, exactly as
    # TireModel::calculate_contact_patch_velocity computes them.
    vx_fl = cos(sa_fl) * v_fl_x + sin(sa_fl) * v_fl_y
    vx_fr = cos(sa_fr) * v_fr_x + sin(sa_fr) * v_fr_y
    vx_rl = cos(sa_rl) * v_rl_x + sin(sa_rl) * v_rl_y
    vx_rr = cos(sa_rr) * v_rr_x + sin(sa_rr) * v_rr_y

    vy_fl = -sin(sa_fl) * v_fl_x + cos(sa_fl) * v_fl_y
    vy_fr = -sin(sa_fr) * v_fr_x + cos(sa_fr) * v_fr_y
    vy_rl = -sin(sa_rl) * v_rl_x + cos(sa_rl) * v_rl_y
    vy_rr = -sin(sa_rr) * v_rr_x + cos(sa_rr) * v_rr_y

    # Slip angle in the SIMULATOR's own form: atan2(Vcy, |Vcx|) * sign(Vcx),
    # evaluated in the wheel frame.
    #
    # This must be computed in the wheel frame, NOT as "steering angle minus
    # velocity angle" in the car frame. Those agree while the car is moving, but
    # at a standstill the velocity term goes to zero and the car-frame form
    # leaves the steering angle standing - so the model believed a PARKED car
    # with 0.2 rad of lock had 0.2 rad of slip angle on both front tyres, and
    # duly produced 0.8 g of lateral force and 15 rad/s^2 of yaw acceleration
    # out of nothing. That is what made the solver spray garbage at a standstill,
    # made every start unstable, and got worse with a longer horizon (more of the
    # horizon sits in the bogus low-speed regime). In the wheel frame both Vcx
    # and Vcy vanish together, so the slip angle correctly goes to zero.
    vx_fl_ang = sqrt(vx_fl**2 + eps_vx_angle**2)
    vx_fr_ang = sqrt(vx_fr**2 + eps_vx_angle**2)
    vx_rl_ang = sqrt(vx_rl**2 + eps_vx_angle**2)
    vx_rr_ang = sqrt(vx_rr**2 + eps_vx_angle**2)

    slip_angle_fl = atan2(vy_fl, vx_fl_ang) * (vx_fl / vx_fl_ang)
    slip_angle_fr = atan2(vy_fr, vx_fr_ang) * (vx_fr / vx_fr_ang)
    slip_angle_rl = atan2(vy_rl, vx_rl_ang) * (vx_rl / vx_rl_ang)
    slip_angle_rr = atan2(vy_rr, vx_rr_ang) * (vx_rr / vx_rr_ang)

    vx_fl_reg = sqrt(vx_fl**2 + eps_vx_ratio**2)
    vx_fr_reg = sqrt(vx_fr**2 + eps_vx_ratio**2)
    vx_rl_reg = sqrt(vx_rl**2 + eps_vx_ratio**2)
    vx_rr_reg = sqrt(vx_rr**2 + eps_vx_ratio**2)

    omega_fl = wheel_speed_scale * x[W_FL]
    omega_fr = wheel_speed_scale * x[W_FR]
    omega_rl = wheel_speed_scale * x[W_RL]
    omega_rr = wheel_speed_scale * x[W_RR]

    slip_ratio_fl = (wheel_radius * omega_fl - vx_fl) / vx_fl_reg
    slip_ratio_fr = (wheel_radius * omega_fr - vx_fr) / vx_fr_reg
    slip_ratio_rl = (wheel_radius * omega_rl - vx_rl) / vx_rl_reg
    slip_ratio_rr = (wheel_radius * omega_rr - vx_rr) / vx_rr_reg

    # --- Aerodynamics -------------------------------------------------------
    drag = 0.5 * air_density * drag_coefficient * frontal_area * vx**2
    downforce = 0.5 * air_density * lift_coefficient * frontal_area * vx**2
    d_force_front = downforce * aero_balance_front
    d_force_rear = downforce * (1.0 - aero_balance_front)

    # --- Load transfer ------------------------------------------------------
    longitudinal_weight_transfer = longitudinal_transfer_gain * x[AX]
    lateral_weight_transfer_front = lateral_transfer_gain_front * x[AY]
    lateral_weight_transfer_rear = lateral_transfer_gain_rear * x[AY]

    static_front_axle = mass * gravity_acceleration * front_mass_distribution
    static_rear_axle = mass * gravity_acceleration * (1.0 - front_mass_distribution)

    vertical_load_fl = (static_front_axle / 2 + d_force_front / 2
                        - longitudinal_weight_transfer / 2 - lateral_weight_transfer_front)
    vertical_load_fr = (static_front_axle / 2 + d_force_front / 2
                        - longitudinal_weight_transfer / 2 + lateral_weight_transfer_front)
    vertical_load_rl = (static_rear_axle / 2 + d_force_rear / 2
                        + longitudinal_weight_transfer / 2 - lateral_weight_transfer_rear)
    vertical_load_rr = (static_rear_axle / 2 + d_force_rear / 2
                        + longitudinal_weight_transfer / 2 + lateral_weight_transfer_rear)

    # A tyre that leaves the ground cannot pull the car down; keep the load
    # non-negative and smooth so the QP never sees a negative-grip tyre.
    def clamp_load(load):
        return 0.5 * (load + sqrt(load**2 + 1.0))

    vertical_load_fl = clamp_load(vertical_load_fl)
    vertical_load_fr = clamp_load(vertical_load_fr)
    vertical_load_rl = clamp_load(vertical_load_rl)
    vertical_load_rr = clamp_load(vertical_load_rr)

    # --- Tyre forces --------------------------------------------------------
    # The slip angles are now in the simulator's own convention, in which Fy is
    # ANTI-correlated with the slip angle (its telemetry shows exactly that), so
    # the lateral force carries a leading minus sign.
    fy_fl = -vertical_load_fl * pacejka(tire_lateral_B, tire_lateral_C, tire_lateral_D,
                                       tire_lateral_E, slip_angle_fl)
    fy_fr = -vertical_load_fr * pacejka(tire_lateral_B, tire_lateral_C, tire_lateral_D,
                                       tire_lateral_E, slip_angle_fr)
    fy_rl = -vertical_load_rl * pacejka(tire_lateral_B, tire_lateral_C, tire_lateral_D,
                                       tire_lateral_E, slip_angle_rl)
    fy_rr = -vertical_load_rr * pacejka(tire_lateral_B, tire_lateral_C, tire_lateral_D,
                                       tire_lateral_E, slip_angle_rr)

    # Pure magic formula, with no linear low-speed branch.
    #
    # Blending in a linear branch below ~5 m/s (as this model used to) is
    # actively harmful: the linear law is unbounded in slip ratio, so near
    # standstill - exactly where the regularised slip ratio gets large - it
    # returns tens of kN per tyre with Jacobians to match, and the QP cannot
    # cope. The magic formula saturates at mu*Fz everywhere, and eps_vx already
    # keeps the slip ratio finite at zero speed.
    def longitudinal_force(load, slip_ratio):
        return load * pacejka(tire_B, tire_C, tire_D, tire_E, slip_ratio)

    fx_fl = longitudinal_force(vertical_load_fl, slip_ratio_fl)
    fx_fr = longitudinal_force(vertical_load_fr, slip_ratio_fr)
    fx_rl = longitudinal_force(vertical_load_rl, slip_ratio_rl)
    fx_rr = longitudinal_force(vertical_load_rr, slip_ratio_rr)

    # --- Combined slip -----------------------------------------------------
    # The two magic formulas above are PURE-slip laws: on their own they let a
    # tyre produce full lateral force while simultaneously producing full
    # longitudinal force. The simulator's MF6.2 applies the proper combined-slip
    # weighting, so without this the controller believes it can brake hard at the
    # rear axle in the middle of a corner at no cost to grip - it commands full
    # regen mid-corner, the real rear axle saturates, and the car spins.
    #
    # Scaling both components by the friction ellipse reproduces the coupling
    # closely enough and costs only a few operations per tyre.
    def combined(fx, fy, load):
        rho_sq = (fx / (tire_D * load)) ** 2 + (fy / (tire_lateral_D * load)) ** 2
        rho = sqrt(rho_sq + 1e-6)
        # Smooth max(1, rho): leaves sub-limit forces untouched and scales
        # anything beyond the ellipse back onto it.
        denom = 0.5 * (1.0 + rho + sqrt((1.0 - rho) ** 2 + 1e-1))
        return fx / denom, fy / denom

    fx_fl, fy_fl = combined(fx_fl, fy_fl, vertical_load_fl)
    fx_fr, fy_fr = combined(fx_fr, fy_fr, vertical_load_fr)
    fx_rl, fy_rl = combined(fx_rl, fy_rl, vertical_load_rl)
    fx_rr, fy_rr = combined(fx_rr, fy_rr, vertical_load_rr)

    # Rolling resistance acts on the wheel as a moment (as in FSFEUP02, which
    # subtracts the tyre's MY from the wheel torque), not on the chassis force.
    rolling_moment_fl = rolling_resistance_coefficient * vertical_load_fl * wheel_radius * smooth_sign(omega_fl, 10.0)
    rolling_moment_fr = rolling_resistance_coefficient * vertical_load_fr * wheel_radius * smooth_sign(omega_fr, 10.0)
    rolling_moment_rl = rolling_resistance_coefficient * vertical_load_rl * wheel_radius * smooth_sign(omega_rl, 10.0)
    rolling_moment_rr = rolling_resistance_coefficient * vertical_load_rr * wheel_radius * smooth_sign(omega_rr, 10.0)

    # --- Chassis forces -----------------------------------------------------
    fx_car_fl = fx_fl * cos(sa_fl) - fy_fl * sin(sa_fl)
    fx_car_fr = fx_fr * cos(sa_fr) - fy_fr * sin(sa_fr)
    fx_car_rl = fx_rl * cos(sa_rl) - fy_rl * sin(sa_rl)
    fx_car_rr = fx_rr * cos(sa_rr) - fy_rr * sin(sa_rr)

    fy_car_fl = fx_fl * sin(sa_fl) + fy_fl * cos(sa_fl)
    fy_car_fr = fx_fr * sin(sa_fr) + fy_fr * cos(sa_fr)
    fy_car_rl = fx_rl * sin(sa_rl) + fy_rl * cos(sa_rl)
    fy_car_rr = fx_rr * sin(sa_rr) + fy_rr * cos(sa_rr)

    total_fx = fx_car_fl + fx_car_fr + fx_car_rl + fx_car_rr - drag
    total_fy = fy_car_fl + fy_car_fr + fy_car_rl + fy_car_rr

    # arm_y is +track/2 for the left wheels, matching FSFEUP02's moment sum
    # (arm_x * fy_vehicle - arm_y * fx_vehicle).
    yaw_moment = (lf * fy_car_fl - s * fx_car_fl
                  + lf * fy_car_fr + s * fx_car_fr
                  - lr * fy_car_rl - s * fx_car_rl
                  - lr * fy_car_rr + s * fx_car_rr)

    # Body-frame velocity derivatives, exactly as the simulator computes them.
    vx_dot = total_fx / mass + vy * yaw_rate
    vy_dot = total_fy / mass - vx * yaw_rate
    yaw_rate_dot = yaw_moment / Izz

    # --- Powertrain ---------------------------------------------------------
    omega_motor = gear_ratio * (omega_rl + omega_rr) / 2.0
    torque_limit = smooth_min(max_motor_torque,
                              max_motor_power / (smooth_abs(omega_motor, 1.0) + 10.0),
                              eps=5.0)
    motor_torque = x[THROTTLE_APPLIED] * torque_limit
    shaft_torque = (motor_torque
                    - viscous_drag_coeff * omega_motor
                    - coulomb_drag * smooth_sign(omega_motor, coulomb_smooth_stiffness))
    axle_torque = shaft_torque * transmission_efficiency * gear_ratio

    # Salisbury differential: a preload plus a torque-proportional ramp caps how
    # much torque can be shuffled between the two rear wheels.
    locking_torque = diff_preload + diff_drive_ramp * smooth_abs(axle_torque, 1.0)
    delta_torque = locking_torque * tanh(diff_kv * (omega_rl - omega_rr) / locking_torque)

    torque_fl = 0.0
    torque_fr = 0.0
    torque_rl = axle_torque / 2.0 - delta_torque
    torque_rr = axle_torque / 2.0 + delta_torque

    alpha_fl = (torque_fl - wheel_radius * fx_fl - rolling_moment_fl
                - front_bearing_drag * omega_fl) / front_wheel_inertia
    alpha_fr = (torque_fr - wheel_radius * fx_fr - rolling_moment_fr
                - front_bearing_drag * omega_fr) / front_wheel_inertia
    alpha_rl = (torque_rl - wheel_radius * fx_rl - rolling_moment_rl) / rear_wheel_inertia
    alpha_rr = (torque_rr - wheel_radius * fx_rr - rolling_moment_rr) / rear_wheel_inertia

    # --- Assembled dynamics -------------------------------------------------
    f_expl = vertcat(
        vx * cos(x[YAW]) - vy * sin(x[YAW]),
        vx * sin(x[YAW]) + vy * cos(x[YAW]),
        yaw_rate,
        vx_dot,
        vy_dot,
        yaw_rate_dot,
        # AX/AY feed the load transfer, so they must hold the specific force the
        # chassis feels, not dv/dt.  Through a steady corner dvy/dt is ~0 while the
        # car is pulling lateral g, so using dv/dt applied almost no lateral load
        # transfer.  Mirrors the same correction in FSFEUP02Model.
        (total_fx / mass - x[AX]) / acceleration_filter_tau,
        (total_fy / mass - x[AY]) / acceleration_filter_tau,
        (x[STEER_CMD] - steering) / steering_motor_tau,
        alpha_fl / wheel_speed_scale,
        alpha_fr / wheel_speed_scale,
        alpha_rl / wheel_speed_scale,
        alpha_rr / wheel_speed_scale,
        u[0],
        u[1],
        (x[THROTTLE_CMD] - x[THROTTLE_APPLIED]) / inverter_tau,
    )

    model.x = x
    model.xdot = xdot
    model.u = u
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl

    # Stability envelope: bound the REAR slip angles.
    #
    # The combined-slip tyre model already caps the force a tyre can make, so the
    # optimizer cannot invent grip. What it CAN still do is plan a trajectory
    # that runs the rear axle past its grip peak, where the tyre gives back force
    # as slip grows - the model happily predicts that, and the result is a
    # planned spin. Bounding the rear slip angles makes that region infeasible.
    # Only the rear is bounded: front saturation is understeer, which is
    # self-correcting and is already handled by the contouring cost.
    model.con_h_expr = vertcat(slip_angle_rl, slip_angle_rr)
    return model


def setup_cost_function(ocp: AcadosOcp):
    """
    Path-tracking objective with a dynamics-decided speed.

    Reference per stage: p = [x_ref, y_ref, v_ref, theta_ref, v_stretch, v_cap].
    """
    x = ocp.model.x
    u = ocp.model.u
    p = ocp.model.p

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    heading_error = sin(x[YAW] - p[P_YAW])

    # Split the position error in the reference point's own frame.
    #
    # This is the change that decouples "where" from "how fast". A single
    # Euclidean position error conflates the two: falling behind the reference
    # POINT costs exactly as much as drifting off the path, so the only way to
    # keep the cost low is to match the planner's timing. Resolving the error
    # along and across the path lets those be weighted independently.
    dx = x[X_POS] - p[P_X]
    dy = x[Y_POS] - p[P_Y]
    contouring_error = -sin(p[P_YAW]) * dx + cos(p[P_YAW]) * dy   # across the path
    lag_error = cos(p[P_YAW]) * dx + sin(p[P_YAW]) * dy           # along the path

    # Ambition: a mild pull towards a slightly optimistic speed. This is what
    # makes the car take a corner faster than planned when the tyres allow it.
    # It is deliberately weak - it must always lose to the contouring term, so
    # the car only carries the extra speed while it can still hold the line.
    speed_ambition = x[VX] - p[P_V_STRETCH]

    # Ceiling: one-sided, and heavy. Zero below the cap, so the planner exerts
    # no downward pull on speed at all until the car tries to exceed it.
    speed_excess = smooth_relu(x[VX] - p[P_V_CAP])

    cost_expression = vertcat(
        contouring_error,      # Across-path error  (path tracking)
        heading_error,         # Heading error
        lag_error,             # Along-path error   (weak: frees the timing)
        speed_ambition,        # Mild pull towards v_ref * stretch
        speed_excess,          # One-sided ceiling at v_ref * cap
        x[VY],                 # Sideslip regularisation
        x[THROTTLE_CMD],       # Throttle regularisation
        x[STEER_CMD],          # Steering regularisation
        u[0],                  # Throttle rate
        u[1],                  # Steering rate
    )

    cost_expression_e = vertcat(
        contouring_error,
        heading_error,
        lag_error,
        speed_ambition,
    )

    ocp.model.cost_y_expr = cost_expression
    ocp.model.cost_y_expr_e = cost_expression_e

    # [contour, heading, lag, ambition, excess, vy, throttle, steer, thr_rate, str_rate]
    #
    # The ordering of magnitudes is the whole design: contour >> excess >> lag,
    # ambition. Contouring must dominate so the car gives up speed rather than
    # the line; the ceiling must outweigh ambition so the cap actually binds;
    # lag and ambition are small so neither the planner's timing nor the
    # optimism ever overrides the physics.
    weights = np.array([30.0, 10.0, 0.3, 0.25, 40.0, 3.0, 0.05, 0.3, 2.0, 4.0])
    weights_e = np.array([30.0, 10.0, 0.3, 0.25])

    ocp.cost.W = np.diag(weights)
    ocp.cost.W_e = np.diag(weights_e)

    ocp.cost.yref = np.zeros(weights.shape[0])
    ocp.cost.yref_e = np.zeros(weights_e.shape[0])

    ocp.dims.ny = weights.shape[0]
    ocp.dims.ny_e = weights_e.shape[0]


def create_ocp_solver(gen_base_dir: str = "./build/acados", acados_dir: str | None = None, acados_lib_dir: str | None = None):
    # gen_base_dir should point to a folder that will contain the generated
    # acados artifacts. We place the C sources under <gen_base_dir>/c_generated_code
    # and the json model at <gen_base_dir>/acados_ocp_mpc.json
    c_code_dir = os.path.abspath(os.path.join(gen_base_dir, "c_generated_code"))
    json_path = os.path.abspath(os.path.join(gen_base_dir, "acados_ocp_mpc.json"))

    if os.path.exists(c_code_dir):
        shutil.rmtree(c_code_dir)
    if os.path.exists(json_path):
        os.remove(json_path)

    os.makedirs(os.path.dirname(c_code_dir), exist_ok=True)
    os.makedirs(os.path.dirname(json_path), exist_ok=True)

    prediction_horizon_seconds, prediction_horizon_steps, _ = load_mpc_parameters()

    if acados_dir is not None:
        ocp = AcadosOcp(acados_path=acados_dir, acados_lib_path=acados_lib_dir)
    else:
        ocp = AcadosOcp()
    ocp.model = export_mpc_model()
    ocp.code_export_directory = c_code_dir

    ocp.solver_options.N_horizon = prediction_horizon_steps
    ocp.solver_options.tf = prediction_horizon_seconds
    # A single guarded real-time iteration per control period. Full SQP on this
    # stiff model cold-starts into NaN; RTI warm-starts from the previous
    # solution and keeps the per-cycle cost bounded.
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    # Partial-condensing blocks, sized to keep ~5 stages per block.
    #
    # This is the validated value at N=50 (cond_N=10). It is NOT a free choice:
    # at N=50, going to 3 stages per block broke the controller outright, and at
    # N=100 leaving cond_N at 10 (10 stages per block) makes the QP fail even
    # with the car driving straight. Keeping the BLOCK SIZE fixed while the
    # horizon grows is what preserves the QP's numerical character.
    ocp.solver_options.qp_solver_cond_N = max(10, prediction_horizon_steps // 5)
    ocp.solver_options.hpipm_mode = "BALANCE"
    # Iteration budget. Do NOT raise this to chase failures on a long horizon:
    # a failing QP burns the whole budget, and at 200 iterations 10% of cycles
    # overran the 25 ms control period (p95 41 ms, max 88 ms). A long horizon has
    # to be made EASIER, not given more iterations.
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.integrator_type = "IRK"
    # The tyre/wheel-spin dynamics settle in single-digit milliseconds while the
    # shooting interval is ~33 ms, so the implicit integrator has to sub-step and
    # be given enough Newton iterations. With one step and three iterations the
    # stage equations do not converge, and the resulting garbage sensitivities
    # are what drove the QP into ACADOS_MINSTEP.
    ocp.solver_options.sim_method_num_stages = 2
    ocp.solver_options.sim_method_num_steps = 2
    ocp.solver_options.sim_method_newton_iter = 5
    ocp.solver_options.levenberg_marquardt = 1e-1
    ocp.solver_options.ext_fun_compile_flags = "-O3 -march=native -ffast-math"
    ocp.solver_options.nlp_solver_max_iter = 10
    ocp.solver_options.with_batch_functionality = True
    ocp.solver_options.num_threads_in_batch_ext_fun = 4

    ocp.solver_options.nlp_solver_tol_stat = 1e-2
    ocp.solver_options.nlp_solver_tol_eq = 1e-2

    # Initial state constraint (required for set_state logic)
    ocp.constraints.x0 = np.zeros(NX)
    ocp.parameter_values = np.zeros(N_PARAMS)

    setup_cost_function(ocp)

    # Rate inputs are hard-bounded; this is what keeps the commands smooth and
    # replaces the old direct-command bang-bang behaviour.
    ocp.constraints.lbu = np.array([-max_throttle_rate, -max_steering_rate])
    ocp.constraints.ubu = np.array([max_throttle_rate, max_steering_rate])
    ocp.constraints.idxbu = np.array([0, 1])

    # The commands themselves live in the state vector, so the actuator limits
    # become simple linear state bounds instead of nonlinear constraints.
    #
    # The longitudinal velocity is bounded below as well. In autonomous mode the
    # plant maps negative throttle to negative motor torque, so a sustained
    # braking request at a standstill drives the car BACKWARDS - and once the
    # model is predicting reverse motion its slip angles and ratios leave the
    # region the tyre fit is valid in and the solver falls apart.
    # acados/HPIPM require the bounded-state indices in ASCENDING order. They are
    # sorted here rather than written out by hand, because getting this wrong is
    # silent: the bounds keep their given order while the indices get reordered,
    # so each limit lands on the wrong state (the velocity ends up clamped to the
    # throttle's +-1, the steering command to [0, 1000]) and the controller
    # behaves bizarrely with no error reported anywhere.
    bounded_states = [
        (VX, min_longitudinal_velocity, 1e3),
        (THROTTLE_CMD, -1.0, 1.0),
        (STEER_CMD, -max_steering_angle, max_steering_angle),
    ]
    bounded_states.sort(key=lambda entry: entry[0])
    ocp.constraints.idxbx = np.array([entry[0] for entry in bounded_states])
    ocp.constraints.lbx = np.array([entry[1] for entry in bounded_states])
    ocp.constraints.ubx = np.array([entry[2] for entry in bounded_states])

    ocp.constraints.lbx_e = ocp.constraints.lbx
    ocp.constraints.ubx_e = ocp.constraints.ubx
    ocp.constraints.idxbx_e = ocp.constraints.idxbx

    # All three are slackened: the plant clamps the commands anyway and the car
    # can physically be pushed below the velocity floor, so a heavily penalised
    # excursion is harmless - whereas a hard bound lets a bad warm start make the
    # QP outright infeasible, and under SQP_RTI an infeasible QP is not
    # recoverable within the cycle. The penalty has to dominate the tracking cost
    # or the solver simply buys the violation: at weight 4 a 3 m position error
    # is worth ~1000 over the horizon.
    slack_penalty = 1e5
    ocp.constraints.idxsbx = np.array([0, 1, 2])
    ocp.constraints.lsbx = np.zeros(3)
    ocp.constraints.usbx = np.zeros(3)
    # Slack ordering in acados is [sbx..., sh...], so the state-bound penalties
    # come first and the slip-angle envelope second. The envelope penalty is
    # lower: exceeding it should be strongly discouraged, not treated as
    # catastrophically as violating an actuator limit.
    ocp.cost.Zl = np.concatenate([slack_penalty * np.ones(3), envelope_penalty * np.ones(2)])
    ocp.cost.Zu = np.concatenate([slack_penalty * np.ones(3), envelope_penalty * np.ones(2)])
    ocp.cost.zl = np.concatenate([slack_penalty * np.ones(3), envelope_penalty * np.ones(2)])
    ocp.cost.zu = np.concatenate([slack_penalty * np.ones(3), envelope_penalty * np.ones(2)])

    # Rear slip angle envelope. Soft, because the car can be pushed past it by a
    # disturbance and an infeasible QP is not recoverable inside one RTI step;
    # the bounds are re-settable at runtime from C++ so they stay tunable.
    ocp.constraints.lh = np.array([-max_rear_slip_angle, -max_rear_slip_angle])
    ocp.constraints.uh = np.array([max_rear_slip_angle, max_rear_slip_angle])
    ocp.constraints.idxsh = np.array([0, 1])
    ocp.constraints.lsh = np.zeros(2)
    ocp.constraints.ush = np.zeros(2)

    ocp.constraints.idxsbx_e = np.array([0, 1, 2])
    ocp.constraints.lsbx_e = np.zeros(3)
    ocp.constraints.usbx_e = np.zeros(3)
    ocp.cost.Zl_e = slack_penalty * np.ones(3)
    ocp.cost.Zu_e = slack_penalty * np.ones(3)
    ocp.cost.zl_e = slack_penalty * np.ones(3)
    ocp.cost.zu_e = slack_penalty * np.ones(3)

    try:
        solver = AcadosOcpSolver(ocp, json_file=json_path)
        return solver
    except Exception as e:
        print(f"FAIL: {e}")
        return None


if __name__ == "__main__":
    # Allow specifying an output directory so multiple MPCs can coexist
    import argparse

    parser = argparse.ArgumentParser(description="Generate acados C code for MPC")
    parser.add_argument("--out-dir", dest="out_dir", default="./build/acados",
                        help="Base output directory for generated files (contains c_generated_code/ and json)")
    parser.add_argument("--acados-dir", dest="acados_dir", default=None,
                        help="Path to acados install root (contains include/ and lib/)")
    parser.add_argument("--acados-lib-dir", dest="acados_lib_dir", default=None,
                        help="Path to acados libraries (defaults to <acados-dir>/lib)")
    args = parser.parse_args()

    create_ocp_solver(args.out_dir, args.acados_dir, args.acados_lib_dir)
