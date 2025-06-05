import numpy as np

class bicycle_model:
    
    def __init__(self, wheelbase=2.5):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.t_prev = None
        self.wheelbase = wheelbase
    
    # Update the state of the bicycle model based on velocity and steering angle
    def update(self, v, delta, t):
        if self.t_prev is None:
            self.t_prev = t
            dt = t
        else:
            dt = t - self.t_prev
            self.t_prev = t
        
        if dt == 0:
            return

        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += (v / self.wheelbase) * np.tan(delta) * dt