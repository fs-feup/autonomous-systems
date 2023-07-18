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
        
        # Perfomance variables
        self.MAX_SPEED = 30.5  # m/s  eq. to 4000 rpm
        self.MAX_ACC = 2.0  # m/ss
        self.MAX_D_ACC = 1.0  # m/sss
        self.MAX_STEER = np.radians(21)  # rad
        self.MAX_D_STEER = np.radians(30)  # rad/s - assumption
        self.VEL = 1.5  # m/s - reference velocity
        self.MAX_TORQUE = 150.0  # Nm
        self.MAX_BREAK = 100

        # Vehicle specification
        self.L = 0.3  # vehicle wheelbase
        self.Lc = self.L / 2  # Disctance from rear axle to vehicle center
        self.tire_diam = 0.51  # m
        self.car_width = 1.201  # m
        self.car_mass = 200  # Kg
        self.power_train_efficiency = 0.85
        self.gear_ratio = 3.5
        self.torque_adjuster = 1.
        self.break_adjuster = 1.
        self.cicles_to_achieve_speed = 2
        self.neg_acceleration_max_break = 5  # m/ss 

        # PID variables
        self.START_BREAKING_POS = 4  # neg index where car starts breaking (PID) 
        self.LOOK_AHEAD = 0  # cte look-ahead (PID)
        self.kp_torque = \
            0.5 * self.torque_adjuster * (self.car_mass * (0.5 * self.tire_diam)) / \
                (self.gear_ratio * self.power_train_efficiency * \
                    self.cicles_to_achieve_speed*self.DT)
        self.kd_torque = 0
        self.kp_break = self.break_adjuster * self.MAX_BREAK / \
            (self.neg_acceleration_max_break * self.cicles_to_achieve_speed * self.DT)
        self.kd_break = 0
        self.kp_steer = 0.5
        self.kd_steer = 8
        self.kp_acc = 1 / (self.cicles_to_achieve_speed * self.DT)
        self.kd_acc = 0

        # distance to final point to trigger ControlNode.done
        self.done_trigger_dist = 0.5  # m