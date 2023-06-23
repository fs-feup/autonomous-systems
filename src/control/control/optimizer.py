import numpy as np
import cvxpy as opt
from .config import Params

np.seterr(divide="ignore", invalid="ignore")

P = Params()    

class Optimizer:
    def __init__(self, N, M, Q, R):
        """ """
        self.state_len = N
        self.action_len = M
        self.state_cost = Q
        self.action_cost = R

    def optimize_linearized_model(
        self,
        A,
        B,
        C,
        initial_state,
        target,
        time_horizon=10,
        Q=None,
        R=None,
        verbose=False,
    ):
        """
        Optimisation problem defined for the linearised model,
        :param A: model matrix A'
        :param B: model matrix B'
        :param C: model matrix C'
        :param initial_state: car's initial state
        :param Q: state cost
        :param R: action cost
        :param target: target state for each timestep in prediction horizon
        :param time_horizon: prediction horizon
        :param verbose: verbose for optimization problem
        :return: optimized states and actions
        """

        assert len(initial_state) == self.state_len

        if Q is None or R is None:
            Q = self.state_cost
            R = self.action_cost

        # Create variables
        x = opt.Variable((self.state_len, time_horizon + 1), name="states")
        u = opt.Variable((self.action_len, time_horizon), name="actions")

        # Loop through the entire time_horizon and append costs
        cost_function = []

        for t in range(time_horizon):

            _cost = opt.quad_form(target[:, t + 1] - x[:, t + 1], Q) + opt.quad_form(
                u[:, t], R
            )

            _constraints = [
                x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C,
                u[0, t] >= -P.MAX_SPEED,
                u[0, t] <= P.MAX_SPEED,
                u[1, t] >= -P.MAX_STEER,
                u[1, t] <= P.MAX_STEER,
            ]

            # Actuation rate of change
            if t < (time_horizon - 1):
                _cost += opt.quad_form(u[:, t + 1] - u[:, t], R * 1)
                _constraints += [opt.abs(u[0, t + 1] - u[0, t]) / P.DT <= P.MAX_ACC]
                _constraints += [opt.abs(u[1, t + 1] - u[1, t]) / P.DT <= P.MAX_D_STEER]

            if t == 0:
                _constraints += [x[:, 0] == initial_state]

            cost_function.append(
                opt.Problem(opt.Minimize(_cost), constraints=_constraints)
            )

        # Add final cost
        problem = sum(cost_function)

        # Minimize Problem
        problem.solve(verbose=verbose, solver=opt.OSQP)

        return x, u