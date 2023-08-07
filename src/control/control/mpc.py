import numpy as np
from .mpc_utils import (
    compute_path_from_wp,
    get_ref_trajectory,
    get_linear_model_matrices,
)
from .optimizer import Optimizer, Params
import time

P = Params()

class MPC:
    def __init__(self, action, state, path, closest_ind):
        # State for the car mathematical model [x,y,heading]
        self.state = state

        # starting guess
        self.action = action 

        self.opt_u = np.zeros((P.M, P.T))

        # Cost Matrices
        Q = np.diag([30, 30, 30, 20])  # state error cost
        R = np.diag([10, 10])  # input cost

        self.optimizer = Optimizer(P.N, P.M, Q, R)

        # Interpolated Path to follow given waypoints
        self.path = compute_path_from_wp(
            path,
            P.path_tick,
        )

        self.closest_ind = None
        self.old_closest_ind = closest_ind

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
        """!
        @brief MPC run function. Pre-processes the current state, calls the optimizer 
        and save the actions.
        @param self The object pointer.
        """
        # start = time.time()

        # dynamycs w.r.t car frame
        curr_state = np.array([0, 0, self.state[2], 0])

        # State Matrices
        A, B, C = get_linear_model_matrices(curr_state, self.action)
        
        # Get Reference_traj -> inputs are in worldframe
        x_target, u_target, self.closest_ind = get_ref_trajectory(
            self.state, self.path, P.VEL, dl=P.path_tick, old_ind=self.old_closest_ind
        )

        x_mpc, u_mpc = self.optimizer.optimize_linearized_model(
            A,
            B,
            C,
            curr_state,
            x_target,
            u_target,
            time_horizon=P.T,
            verbose=False
        )
        
        self.opt_u = np.vstack(
            (
                np.array(u_mpc.value[0, :]).flatten(),
                (np.array(u_mpc.value[1, :]).flatten()),
            )
        )
        self.action[:] = [u_mpc.value[0, 0], u_mpc.value[1, 0]]
        # print("CVXPY Optimization Time: {:.4f}s".format(time.time()-start))

def run_mpc(action, state, path, old_closest_ind):
    mpc = MPC(action, state, path, old_closest_ind)
    try:
        mpc.run()
        return mpc.action, mpc.closest_ind, len(mpc.path)
    except Exception:
        return None, None, None
