import numpy as np
from .helpers import compute_path_from_wp
from .mpc import MPC, get_linear_model_matrices, get_ref_trajectory, Params
import time

# Robot Starting position
SIM_START_X = 0.0
SIM_START_Y = 0.5
SIM_START_V = 0.0
SIM_START_H = 0.0
L = 0.3

P = Params()

# Params
VEL = 1.0  # m/s


# Classes
class MPCSim:
    def __init__(self, action, state, path):

        # State for the robot mathematical model [x,y,heading]
        self.state = state

        # starting guess
        self.action = action 
        # self.action[0] = node.acceleration 
        # self.action[1] = node.steering_angle

        self.opt_u = np.zeros((P.M, P.T))

        # Cost Matrices
        Q = np.diag([20, 20, 10, 20])  # state error cost
        Qf = np.diag([30, 30, 30, 30])  # state final error cost
        R = np.diag([10, 10])  # input cost
        R_ = np.diag([10, 10])  # input rate of change cost

        self.mpc = MPC(P.N, P.M, Q, R)

        # Interpolated Path to follow given waypoints
        self.path = compute_path_from_wp(
            path,
            P.path_tick,
        )

        # Sim help vars
        self.sim_time = 0
        self.x_history = []
        self.y_history = []
        self.v_history = []
        self.h_history = []
        self.a_history = []
        self.d_history = []
        self.predicted = None

    def run(self):
        """
        [TODO:summary]
        [TODO:description]
        """
        start = time.time()

        # dynamycs w.r.t robot frame
        curr_state = np.array([0, 0, self.state[2], 0])
        # State Matrices
        A, B, C = get_linear_model_matrices(curr_state, self.action)
        # Get Reference_traj -> inputs are in worldframe
        target, _ = get_ref_trajectory(
            self.state, self.path, VEL, dl=P.path_tick
        )

        x_mpc, u_mpc = self.mpc.optimize_linearized_model(
            A,
            B,
            C,
            curr_state,
            target,
            time_horizon=P.T,
            verbose=False,
        )
        self.opt_u = np.vstack(
            (
                np.array(u_mpc.value[0, :]).flatten(),
                (np.array(u_mpc.value[1, :]).flatten()),
            )
        )
        self.action[:] = [u_mpc.value[0, 0], u_mpc.value[1, 0]]
        print("CVXPY Optimization Time: {:.4f}s".format(time.time()-start))

def do_sim(action, state, path):
    sim = MPCSim(action, state, path)
    try:
        sim.run()
        return sim.action
    except Exception as e:
        return None
