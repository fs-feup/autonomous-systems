import numpy as np


class Params:
    def __init__(self):
        # enviroment choice
        self.env = "eufs"

        # MPC variables
        self.N = 3  # number of state variables
        self.M = 2  # number of control variables
        self.T = 10  # Prediction Horizon
        self.DT = 0.2  # discretization step
        self.path_tick = 0.05  # average distance btw path points

        # PID variables
        self.START_BREAKING_POS = 12  # neg index where car starts breaking (PID) 
        self.LOOK_AHEAD = 0  # cte look-ahead (PID)
        
        # Perfomance variables
        self.MAX_SPEED = 1.5  # m/s
        self.MAX_ACC = 1.0  # m/ss
        self.MAX_D_ACC = 1.0  # m/sss
        self.MAX_STEER = np.radians(30)  # rad
        self.MAX_D_STEER = np.radians(30)  # rad/s
        self.VEL = 1.5  # m/s - reference velocity

        # Vehicle specification
        self.L = 0.3  # vehicle wheelbase
        self.Lc = self.L / 2  # Disctance from rear axle to vehicle center
        self.tire_diam = 0.51  # m
        self.car_width = 1.201  # m