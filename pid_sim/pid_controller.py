from math import *
class PIDController:
    """ single PID controller """
    def __init__(self, p, i, d, dead_band, limit):
        self.kp = p
        self.ki = i
        self.kd = d
        self.dead_band = dead_band
        self.output = 0.
        self.prev_error = 0.
        self.prev_d_error = 0.
        self.limit = limit

    def update(self, actual, desire):
        curr_error = desire - actual
        curr_d_error = curr_error - self.prev_error
        out_i = self.ki * curr_error
        out_p = self.kp * curr_d_error
        out_d = self.kd * (curr_d_error - self.prev_d_error)
        
        if(abs(curr_error) > self.dead_band):
            self.output += (out_p + out_i + out_d)

        if self.output > self.limit[1]:
            self.output = self.limit[1]
        
        if self.output < self.limit[0]:
            self.output = self.limit[0]

        self.prev_error = curr_error
        self.prev_d_error = curr_d_error

        return self.output
