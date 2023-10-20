import numpy as np
import datetime
import csv
from .mpc_utils import (
    compute_path_from_wp,
    get_ref_trajectory,
    get_linear_model_matrices,
    optimize,
)

from .config import Params

P = Params()

class MPC:
    def __init__(self, action, state, path, closest_ind):
        # State for the car mathematical model [x,y,v, orientation]
        self.state = state

        # starting guess -> last command [steering angle, acceleration]
        self.action = action 

        self.opt_u = np.zeros((P.command_len, P.prediction_horizon))

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

    def run(self, isTesting = False):
        """!
        @brief MPC run function. Pre-processes the current state, calls the optimizer 
        and save the actions.
        @param self The object pointer.
        """

        # start = time.time()

        # (x, y, vel, heading) state - car frame
        curr_state = np.array([0, 0, self.state[2], 0])

        # State Matrices
        A, B, C = get_linear_model_matrices(curr_state, self.action)
        
        # Get Reference_Trajectory Values (Inputs -> Worldframe - Outputs - Carframe)
        x_target, u_target, self.closest_ind = get_ref_trajectory(
            self.state, self.path, P.VEL, old_ind=self.old_closest_ind
        )

        t0 = datetime.datetime.now()  

        x_mpc, u_mpc = optimize(
            A,
            B,
            C,
            curr_state,
            x_target,
            u_target,
            verbose=False
        )

        t1 = datetime.datetime.now()

        dt = t1 - t0

        if isTesting:
            dtsum = 0
            no_iters = 100
            for i in range(no_iters):

                t0 = datetime.datetime.now()  

                x_mpc, u_mpc = optimize(
                    A,
                    B,
                    C,
                    curr_state,
                    x_target,
                    u_target,
                    verbose=False
                )

                t1 = datetime.datetime.now()

                dtsum += (t1 - t0).microseconds

            with open('control/test/control_measures.csv', 'a') as f:
                writer = csv.writer(f)
                writer.writerow(['control', 'mpc',\
                    'optimization_step-' + str(P.prediction_horizon) + 'ph',\
                    dtsum / no_iters * 1e-3, dt.microseconds * 1e-3])
            
            print("Average optimization step is ", dtsum / no_iters * 1e-3)
        
        self.get_logger().debug("Otimization step calculated in ",\
            dt.microseconds * 1e-3, " ms")

        
        self.opt_u = np.vstack(
            (
                np.array(u_mpc.value[0, :]).flatten(),
                (np.array(u_mpc.value[1, :]).flatten()),
            )
        )
        # Select the first action only - the one to be performed in current instant
        self.action[:] = [u_mpc.value[0, 0], u_mpc.value[1, 0]]
        # print("CVXPY Optimization Time: {:.4f}s".format(time.time()-start))

def run_mpc(action, state, path, old_closest_ind, isTesting):
    mpc = MPC(action, state, path, old_closest_ind)
    try:
        mpc.run(isTesting)
        return mpc.action, mpc.closest_ind, len(mpc.path)
    except Exception:
        return None, None, None
