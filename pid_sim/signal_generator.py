class PWMGenerator:
    def __init__(self, time_step, frequency, duty):
        self.period = 1./frequency
        self.duty = duty
        self.t = 0
        self.time_step = time_step

    def update(self):
        self.t += self.time_step
        if(self.t > self.period):
            self.t -= self.period
        return 1 if (self.t / self.period) < self.duty else 0
