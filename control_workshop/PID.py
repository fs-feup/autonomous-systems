class PID:
    #ut = Kp * et + Ki * sum(et) + Kd * (et - et_1) / dt
    #et = setpoint - measurement
    #Kp  # Proportional gain
    #Ki # Integral gain
    
    def __init__(self, Kp=1.0, Ki=0.1, Kd=0.05):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.previous_error = 0.0
        
    def reset(self, Kp=None, Ki=None, Kd=None):
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd
        self.integral = 0.0
        self.previous_error = 0.0
    
    
    def update(self, setpoint, measurement, dt):
        e_t = setpoint - measurement
        self.integral += e_t * dt
        
        Kp_component = self.Kp * e_t
        Ki_component = self.Ki * self.integral
        
        if dt > 0:
            Kd_component = self.Kd * ((e_t - self.previous_error) / dt)
        else:
            Kd_component = 0.0

        self.previous_error = e_t

        return Kp_component + Ki_component + Kd_component