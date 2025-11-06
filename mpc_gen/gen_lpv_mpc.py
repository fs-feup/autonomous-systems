# file: mpc_gen/gen_lpv_mpc.py
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import os

# --- MPC dimensions ---
nx = 4             # [ey, epsi, vy, r]
nu = 1             # [delta]
N  = 100
dt = 0.01

# LPV parameters per stage: Ad (16) + Bd (4) + g (4) = 24
nA = nx*nx
nB = nx*nu
ng = nx
np_param = nA + nB + ng  # 24

def build_model():
    model = AcadosModel()
    model.name = "lpv_mpc"

    ey   = ca.SX.sym('ey')
    epsi = ca.SX.sym('epsi')
    vy   = ca.SX.sym('vy')
    r    = ca.SX.sym('r')
    x = ca.vertcat(ey, epsi, vy, r)

    delta = ca.SX.sym('delta')  # input
    u = delta

    p = ca.SX.sym('p', np_param)  # parameters

    Ad_vec = p[0:nA]
    Bd_vec = p[nA:nA+nB]
    g_vec  = p[nA+nB:nA+nB+ng]

    Ad = ca.reshape(Ad_vec, nx, nx)
    Bd = ca.reshape(Bd_vec, nx, 1)
    g  = ca.reshape(g_vec,  nx, 1)

    # discrete dynamics
    x_next = Ad @ x + Bd @ u + g

    model.x = x
    model.u = u
    model.p = p
    model.disc_dyn_expr = x_next
    return model

def build_ocp():
    ocp = AcadosOcp()
    ocp.model = build_model()

    # default params required by acados_template
    ocp.parameter_values = np.zeros((np_param, ))

    # Weights
    W_EY, W_EPSI, W_DELTA, W_SLACK = 1000.0, 10.0, 1.0, 1000.0

    # y = [ey, epsi, delta, s_placeholder]
    ny = 4
    Vx = np.zeros((ny, nx))
    Vu = np.zeros((ny, nu))
    Vx[0, 0] = 1.0  # ey
    Vx[1, 1] = 1.0  # epsi
    Vu[2, 0] = 1.0  # delta

    ocp.cost.cost_type   = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.Vx   = Vx
    ocp.cost.Vu   = Vu
    ocp.cost.Vx_e = Vx[:2, :]
    ocp.cost.Vu_e = np.zeros((2, nu))

    ocp.cost.W   = np.diag([W_EY, W_EPSI, W_DELTA, 0.0])
    ocp.cost.W_e = np.diag([W_EY, W_EPSI])
    ocp.cost.yref   = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(2)

    # Constraints
    DELTA_MAX = np.deg2rad(20.0)
    TRACK_HALF_WIDTH = 1.0

    # input bounds
    ocp.constraints.lbu   = np.array([-DELTA_MAX])
    ocp.constraints.ubu   = np.array([ DELTA_MAX])
    ocp.constraints.idxbu = np.array([0], dtype=np.int64)

    # general linear: C*x + D*u <= ug ; >= lg  (NO INFINITIES IN JSON!)
    C = np.zeros((2, nx))
    D = np.zeros((2, nu))
    C[0,0] =  1.0  #  ey <= +W
    C[1,0] = -1.0  # -ey <= +W  (i.e., ey >= -W)
    ocp.constraints.C = C
    ocp.constraints.D = D
    BIG = 1e9
    ocp.constraints.lg = np.array([-BIG, -BIG])                     # was [-inf, -inf]
    ocp.constraints.ug = np.array([ TRACK_HALF_WIDTH, TRACK_HALF_WIDTH])

    # soften those 2 general constraints
    ocp.constraints.idxsg = np.array([0, 1], dtype=np.int64)
    ns = ocp.constraints.idxsg.size
    ocp.constraints.lsg = np.zeros(ns)
    ocp.constraints.usg = np.zeros(ns)
    ocp.cost.zl = W_SLACK*np.ones(ns)   # L1
    ocp.cost.zu = W_SLACK*np.ones(ns)
    ocp.cost.Zl = np.zeros(ns)          # L2
    ocp.cost.Zu = np.zeros(ns)

    # also set node-0 slack fields to silence the info message
    ocp.cost.zl_0 = ocp.cost.zl
    ocp.cost.zu_0 = ocp.cost.zu
    ocp.cost.Zl_0 = ocp.cost.Zl
    ocp.cost.Zu_0 = ocp.cost.Zu

    # Horizon & options
    ocp.solver_options.tf = N * dt
    ocp.solver_options.N_horizon = N
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'DISCRETE'
    ocp.solver_options.qp_solver_iter_max = 100
    ocp.solver_options.qp_tol = 1e-6
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    return ocp

if __name__ == "__main__":
    # (optional) pin a compatible tera if your libc is old
    try:
        from acados_template.utils import get_tera
        get_tera(tera_version='0.0.34', force_download=False)
    except Exception as e:
        print("tera pre-check: ", e)

    ocp = build_ocp()

    JSON_DIR = os.path.abspath("./acados_lpv_build")
    os.makedirs(JSON_DIR, exist_ok=True)

    ocp_solver = AcadosOcpSolver(ocp, json_file=f"{JSON_DIR}/ocp.json")
    print("Generated acados solver in:", JSON_DIR)
