import numpy as np
import scipy.sparse as sp
import osqp

# --- Vehicle parameters ---
m   = 240.0          # kg
Iz  = 101.082        # kg*m^2
lf  = 0.726          # m
lr  = 0.804          # m
L   = lf + lr        # wheelbase

# Cornering stiffness (axle) initial guesses (tune later)
Cf  = 2.27e4         # N/rad  (front axle)
Cr  = 2.05e4         # N/rad  (rear axle)

# Limits
delta_max  = np.deg2rad(20.0)   # steering limit (±20°)
ddelta_max = np.deg2rad(40.0)   # steering rate (40°/s)
a_x_max    = 8.0                # m/s^2  (throttle = +1 → +8 m/s²)
a_x_min    = -8.0               # m/s^2  (throttle = -1 → -8 m/s²)

track_half_width = 0.50         # soft track bound (±0.5 m around path)

# MPC grid (you asked N=50, dt=0.04)
N  = 50
dt = 0.04

# Weights (start here; tune later)
w_ey     = 10.0
w_epsi   = 6.0
w_vx     = 1.0
w_delta  = 0.5
w_ddelta = 6.0
w_ax     = 0.05
w_dax    = 0.5
w_slack  = 50.0


def finite_diff_heading_and_curvature(xs, ys):
    xs = np.asarray(xs); ys = np.asarray(ys)
    dx = np.gradient(xs)
    dy = np.gradient(ys)
    heading = np.arctan2(dy, dx)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = (dx*dx + dy*dy)**1.5 + 1e-9
    kappa = (dx*ddy - dy*ddx) / denom
    return heading, kappa

def wrap_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def project_pose_to_path(x, y, xs, ys):
    """Find nearest segment on path to (x,y) and compute s-index."""
    pts = np.vstack([xs, ys]).T
    diffs = pts - np.array([x, y])
    dists2 = np.sum(diffs*diffs, axis=1)
    idx = int(np.argmin(dists2))
    return idx

def local_errors(x, y, yaw, xs, ys, psi_path):
    """Compute lateral error e_y and heading error e_psi relative to nearest path point."""
    idx = project_pose_to_path(x, y, xs, ys)
    px, py, psi = xs[idx], ys[idx], psi_path[idx]
    # Rotate into path frame
    R = np.array([[np.cos(psi), np.sin(psi)],
                  [-np.sin(psi), np.cos(psi)]])
    e = R @ (np.array([x - px, y - py]))
    ey = e[1]  # lateral
    epsi = wrap_angle(yaw - psi)
    return ey, epsi, idx

def build_AB(vx):
    """Continuous-time A, B for lateral bicycle model at speed vx (LPV). States: [ey, epsi, vy, r, vx]. Inputs: [delta, ax]."""
    # Guard low speeds
    vx_eff = max(0.5, float(vx))

    A = np.zeros((5,5))
    B = np.zeros((5,2))
    # ey_dot = vy + vx * epsi
    A[0,1] = vx_eff
    A[0,2] = 1.0

    # epsi_dot = r - vx*kappa -> kappa handled as affine term outside (g_k)
    A[1,3] = 1.0

    # vy_dot
    A[2,2] = -(Cf + Cr)/(m*vx_eff)
    A[2,3] = -(Cf*lf - Cr*lr)/(m*vx_eff)
    B[2,0] = Cf/m  # delta

    # r_dot
    A[3,2] = -(Cf*lf - Cr*lr)/(Iz*vx_eff)
    A[3,3] = -(Cf*lf*lf + Cr*lr*lr)/(Iz*vx_eff)
    B[3,0] = Cf*lf/Iz  # delta

    # vx_dot = ax
    B[4,1] = 1.0  # ax

    # Discretize (Euler or expm; Euler is fine for small dt)
    Ad = np.eye(5) + A*dt
    Bd = B*dt
    return Ad, Bd

def stack_prediction_matrices(Ad_list, Bd_list, g_list):
    """Build big sparse matrices for horizon dynamics: X_{k+1} = Ad_k X_k + Bd_k U_k + g_k."""
    nx = Ad_list[0].shape[0]
    nu = Bd_list[0].shape[1]

    # Decision vector z = [x0, x1,..., xN, u0,...,u_{N-1}, slack0..slackN]^T
    # We'll place slacks only on ey constraints per stage (N+1 slacks).
    Ns = N+1

    # Indices helpers
    def ix(k): return k*nx
    def iu(k): return (N+1)*nx + k*nu
    slack_base = (N+1)*nx + N*nu

    # Dynamics equality: for k=0..N-1, x_{k+1} - Ad_k x_k - Bd_k u_k = g_k
    rows = []
    cols = []
    data = []
    b = []

    row = 0
    for k in range(N):
        Ad = Ad_list[k]; Bd = Bd_list[k]
        # x_{k+1}
        for i in range(nx):
            rows.append(row + i); cols.append(ix(k+1) + i); data.append(1.0)
        # -Ad_k x_k
        Ak = -Ad
        r, c = np.nonzero(Ak)
        for i, j in zip(r, c):
            rows.append(row + i); cols.append(ix(k) + j); data.append(Ak[i, j])
        # -Bd_k u_k
        Bk = -Bd
        r, c = np.nonzero(Bk)
        for i, j in zip(r, c):
            rows.append(row + i); cols.append(iu(k) + j); data.append(Bk[i, j])
        # RHS = g_k
        b.extend(g_list[k])
        row += nx

    Aeq = sp.csc_matrix((data, (rows, cols)),
                        shape=(N*nx, (N+1)*nx + N*nu + Ns))

    beq = np.array(b)

    # Selector helpers for building objective and inequality constraints outside
    return Aeq, beq, slack_base, nx, nu, Ns

def mpc_solve(current_pose, current_vx, delta_prev, path_points):
    """
    Solve one MPC step.
    current_pose: (x, y, yaw)
    current_vx: float [m/s]
    delta_prev: previous steering [rad]
    path_points: array-like of (x_i, y_i, v_ref_i), length >= N+1
    Returns: (delta_cmd_rad, throttle_cmd in [-1,1])
    """
    x, y, yaw = current_pose
    path_points = np.asarray(path_points)
    xs = path_points[:,0]; ys = path_points[:,1]; vrefs = path_points[:,2]

    # Path heading and curvature over the provided window
    psi_path, kappas = finite_diff_heading_and_curvature(xs, ys)

    # Compute initial errors relative to path and find start index
    ey0, epsi0, idx0 = local_errors(x, y, yaw, xs, ys, psi_path)

    # Build per-stage Ad, Bd with scheduled vx (use vref along horizon; could also propagate predicted vx)
    Ad_list = []
    Bd_list = []
    g_list  = []  # affine terms from curvature in epsi equation, and ey from epsi coupling (already in A)

    nx = 5; nu = 2

    # Initialize nominal state for "known" affine terms (only epsi uses curvature via -vx*kappa*dt)
    for k in range(N):
        v_scheduled = float(vrefs[min(idx0 + k, len(vrefs)-1)])
        Ad, Bd = build_AB(v_scheduled)
        Ad_list.append(Ad); Bd_list.append(Bd)

        gk = np.zeros(nx)
        # epsi_{k+1} = epsi_k + dt*(r_k - vx*kappa_k)  -> move -dt*vx*kappa_k to RHS (affine)
        kappa_k = float(kappas[min(idx0 + k, len(kappas)-1)])
        gk[1] = -dt * v_scheduled * kappa_k
        g_list.append(gk)

    # Assemble dynamics constraints
    Aeq, beq, slack_base, nx, nu, Ns = stack_prediction_matrices(Ad_list, Bd_list, g_list)

    # Decision vector z = [x0..xN, u0..u_{N-1}, s0..sN]
    nz = (N+1)*nx + N*nu + Ns

    # Initial condition equality: x0 = [ey0, epsi0, 0, 0, current_vx]
    x0 = np.array([ey0, epsi0, 0.0, 0.0, float(current_vx)])

    Ainit = sp.lil_matrix((nx, nz))
    Ainit[:, 0:nx] = sp.eye(nx)
    Ainit = Ainit.tocsc()
    binit = x0

    # Inequality constraints: steering bounds, rate bounds, accel bounds, track soft bounds
    Aineq_rows = []
    Aineq_cols = []
    Aineq_data = []
    lb = []
    ub = []

    def add_bound(var_index, lower, upper):
        pass

    A_blocks = [Aeq, Ainit]

    l = np.hstack([beq, binit])
    u = np.hstack([beq, binit])

    # Track soft bounds on ey:  -w <= ey_k <= +w  with slack s_k >= 0, i.e., -w - s_k <= ey_k <= +w + s_k
    rows = []
    cols = []
    data = []
    l_tr = []
    u_tr = []

    for k in range(N+1):
        ey_col = k*nx + 0
        s_col = slack_base + k

        rows.append(k); cols.append(ey_col); data.append(1.0)
        rows.append(k); cols.append(s_col); data.append(1.0)
        u_tr.append(track_half_width)
        l_tr.append(-np.inf)

        rows.append((N+1)+k); cols.append(ey_col); data.append(-1.0)
        rows.append((N+1)+k); cols.append(s_col); data.append(1.0)
        u_tr.append(track_half_width)
        l_tr.append(-np.inf)

    Atrack = sp.csc_matrix((data, (rows, cols)), shape=(2*(N+1), nz))
    A_blocks.append(Atrack)
    l = np.hstack([l, np.array(l_tr)])
    u = np.hstack([u, np.array(u_tr)])

    # Input bounds and rate constraints
    rows=[]; cols=[]; data=[]; l_in=[]; u_in=[]

    for k in range(N):
        delta_col = (N+1)*nx + k*nu + 0
        ax_col    = (N+1)*nx + k*nu + 1

        rows.append(2*k);   cols.append(delta_col); data.append(1.0); u_in.append(delta_max); l_in.append(-np.inf)
        rows.append(2*k+1); cols.append(delta_col); data.append(-1.0); u_in.append(delta_max); l_in.append(-np.inf)

        rows.append(2*N + 2*k);   cols.append(ax_col); data.append(1.0); u_in.append(a_x_max); l_in.append(-np.inf)
        rows.append(2*N + 2*k+1); cols.append(ax_col); data.append(-1.0); u_in.append(-a_x_min); l_in.append(-np.inf)

    rate_rows = []; rate_cols=[]; rate_data=[]; l_rt=[]; u_rt=[]
    rate_lim = ddelta_max * dt
    for k in range(N):
        delta_col = (N+1)*nx + k*nu + 0
        if k == 0:
            rate_rows.append(2*k);   rate_cols.append(delta_col); rate_data.append(1.0)
            u_rt.append(rate_lim + delta_prev); l_rt.append(-np.inf)
            rate_rows.append(2*k+1); rate_cols.append(delta_col); rate_data.append(-1.0)
            u_rt.append(rate_lim - delta_prev); l_rt.append(-np.inf)
        else:
            delta_prev_col = (N+1)*nx + (k-1)*nu + 0
            rate_rows.append(2*k);   rate_cols.append(delta_col); rate_data.append(1.0)
            rate_rows.append(2*k);   rate_cols.append(delta_prev_col); rate_data.append(-1.0)
            u_rt.append(rate_lim); l_rt.append(-np.inf)
            rate_rows.append(2*k+1); rate_cols.append(delta_col); rate_data.append(-1.0)
            rate_rows.append(2*k+1); rate_cols.append(delta_prev_col); rate_data.append(1.0)
            u_rt.append(rate_lim); l_rt.append(-np.inf)

    Ainputs = sp.csc_matrix((data+rate_data, (rows+rate_rows, cols+rate_cols)),
                            shape=(4*N + 2*N, nz))
    A_blocks.append(Ainputs)
    l = np.hstack([l, np.array(l_in + l_rt)])
    u = np.hstack([u, np.array(u_in + u_rt)])

    P_rows=[]; P_cols=[]; P_data=[]
    q = np.zeros(nz)

    for k in range(N+1):
        idx = k*nx + 0
        P_rows.append(idx); P_cols.append(idx); P_data.append(w_ey)
        idx = k*nx + 1
        P_rows.append(idx); P_cols.append(idx); P_data.append(w_epsi)
        idx_vx = k*nx + 4
        P_rows.append(idx_vx); P_cols.append(idx_vx); P_data.append(w_vx)
        s_col = slack_base + k
        P_rows.append(s_col); P_cols.append(s_col); P_data.append(w_slack)

        vref_k = float(vrefs[min(idx0 + k, len(vrefs)-1)])
        q[idx_vx] += -2.0*w_vx*vref_k

    for k in range(N):
        delta_col = (N+1)*nx + k*nu + 0
        ax_col    = (N+1)*nx + k*nu + 1
        P_rows.append(delta_col); P_cols.append(delta_col); P_data.append(w_delta)
        P_rows.append(ax_col);    P_cols.append(ax_col);    P_data.append(w_ax)

        P_rows.append(delta_col); P_cols.append(delta_col); P_data.append(w_ddelta)
        P_rows.append(ax_col);    P_cols.append(ax_col);    P_data.append(w_dax)

        if k > 0:
            delta_prev_col = (N+1)*nx + (k-1)*nu + 0
            ax_prev_col    = (N+1)*nx + (k-1)*nu + 1
            P_rows.append(delta_col); P_cols.append(delta_prev_col); P_data.append(-w_ddelta)
            P_rows.append(delta_prev_col); P_cols.append(delta_col); P_data.append(-w_ddelta)
            P_rows.append(ax_col); P_cols.append(ax_prev_col); P_data.append(-w_dax)
            P_rows.append(ax_prev_col); P_cols.append(ax_col); P_data.append(-w_dax)
        else:
            q[delta_col] += -2.0*w_ddelta*delta_prev
            q[ax_col] += 0.0

    P = sp.csc_matrix((P_data, (P_rows, P_cols)), shape=(nz, nz))

    A = sp.vstack(A_blocks, format='csc')

    prob = osqp.OSQP()
    prob.setup(P=P, q=q, A=A, l=l, u=u, verbose=False, polish=True, eps_abs=1e-4, eps_rel=1e-4, max_iter=4000)
    res = prob.solve()

    if res.info.status_val not in (1, 2):
        delta_cmd = np.clip(delta_prev, -delta_max, delta_max)
        throttle = 0.0
        return float(delta_cmd), float(throttle)

    z = res.x
    u0 = z[(N+1)*nx : (N+1)*nx + 2]
    delta_cmd = float(np.clip(u0[0], -delta_max, +delta_max))
    ax_cmd = float(np.clip(u0[1], a_x_min, a_x_max))
    throttle = float(np.clip(ax_cmd / a_x_max, -1.0, +1.0))

    return delta_cmd, throttle
