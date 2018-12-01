class Delayer:
    def __init__(self, time_step, delay_t):
        self.time_step = time_step
        self.delay_t = delay_t
        self.data_queue = []
        
    def input(self, data):
        self.data_queue += [[data, self.delay_t]]

    def update(self):
        for pair in self.data_queue:
            pair[1] -=  self.time_step

    def read(self):
        if(len(self.data_queue) == 0):
            return 0
        data = self.data_queue[0][0]
        last_output_idx = -1
        for i in range(0,len(self.data_queue)):
            if(self.data_queue[i][1] < 0):
                data = self.data_queue[i][0]   
                last_output_idx = i

        if(last_output_idx >= 0):
            self.data_queue = self.data_queue[i+1:]
        
        return data
