import numpy as np
from delayer import Delayer

class SimpleSystem:
    """ single input output system """
    gravity = 9.8
    def __init__(self, time_step):
        self.friction = 100
        self.altitude = 0.
        self.time_step = time_step
        self.velocity = 0.
        self.mass = 0.05
        # self.input_delayer = Delayer(time_step, 0.015)

    def update(self, input):
        # self.input_delayer.input(input)
        # self.input_delayer.update()
        acc = (input + np.random.normal(0,0.1)) / self.mass# - self.velocity * 10 
        self.velocity += acc * self.time_step
        self.altitude += self.velocity * self.time_step

    def observe(self):
        return self.altitude + np.random.normal(0,0.4)