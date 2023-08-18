import numpy as np

class Params:
    def __init__(self):
        # enviroment choice
        self.env = "eufs"

        # controller choice (either "pid or "mpc")
        self.controller = "pid"

        # MPC variables
        self.state_len = 4  # number of state variables
        self.command_len = 2  # number of control 
        self.state_cost = np.diag([30, 30, 30, 20])  # state error cost
        self.command_cost = np.diag([10, 10])  # command cost
        self.prediction_horizon = 10  # Prediction Horizon (iterations)
        self.DT = 0.2  # discretization step
        self.path_tick = 0.05  # average distance btw path points
        
        # Perfomance variables
        self.MAX_SPEED = 1.0 # m/s
        self.MAX_ACC = 1.0  # m/ss
        self.MAX_D_ACC = 1.0  # m/sss
        self.MAX_STEER = np.radians(21)  # rad
        self.MAX_D_STEER = np.radians(30)  # rad/s - assumption
        self.VEL = 1.0 # m/s - reference velocity
        self.MAX_TORQUE = 50.0  # Nm
        self.MAX_BREAK = 100

        # TODO: test params and approximate them to real life
        # Vehicle specification
        self.L = 0.3  # vehicle wheelbase
        self.Lc = self.L / 2  # Disctance from rear axle to vehicle center
        self.tire_diam = 0.51  # m
        self.car_width = 1.201  # m
        self.car_mass = 200  # Kg
        self.power_train_efficiency = 0.85 # % of power generated received by the wheels  
        self.gear_ratio = 3.5 # ratio between driver(effort) gear and driven(load) gear
        self.torque_adjuster = 1. # aid variable to quickly variate scale
        self.break_adjuster = 1. # aid variable to quickly variate scale
        self.cicles_to_achieve_speed = 7
        self.neg_acceleration_max_break = 5  # m/ss 

        # PID variables
        self.error_list_size = 20
        self.START_BREAKING_POS = 4  # neg index where car starts breaking (PID) 
        self.LOOK_AHEAD = 0  # cte look-ahead (PID)
        self.spline_window = 10
        self.spline_n_new_points = 100
        self.spline_degree = 3
        self.kp_torque = \
            0.5 * self.torque_adjuster * (self.car_mass * (0.5 * self.tire_diam)) / \
                (self.gear_ratio * self.power_train_efficiency)
        self.kp_break = self.break_adjuster * self.MAX_BREAK / \
            (self.neg_acceleration_max_break)
        self.kp_steer = 0.5
        self.kd_steer = 8
        self.kp_acc = 1 / (self.cicles_to_achieve_speed * self.DT)
        self.kd_acc = 0

        # distance to final point to trigger ControlNode.done
        self.done_trigger_dist = 0.5  # m