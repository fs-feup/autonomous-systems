import numpy as np


class Params:
    def __init__(self):
        self.N = 4  # number of state variables
        self.M = 2  # number of control variables
        self.T = 10  # Prediction Horizon
        self.DT = 0.2  # discretization step
        self.path_tick = 0.05  # average distance btw path points
        self.L = 0.3  # vehicle wheelbase
        self.MAX_SPEED = 1.5  # m/s
        self.MAX_ACC = 1.0  # m/ss
        self.MAX_D_ACC = 1.0  # m/sss
        self.MAX_STEER = np.radians(30)  # rad
        self.MAX_D_STEER = np.radians(30)  # rad/s
        self.VEL = 1  # m/s
        self.START_BREAKING_POS = 12  # index (backwards) where car starts breaking (PID) 
        self.LOOK_AHEAD = 0  # cte look-ahead (PID)