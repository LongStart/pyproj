import numpy as np

def CreateKnotVector(degree, time_stamps):
    front_len = (degree + 1) / 2
    back_len = front_len if degree % 2 == 1 else (front_len + 1)
    return np.hstack([[time_stamps[0]]* front_len, time_stamps, [time_stamps[-1]]* back_len])

def CreateUniformKnotVector(degree, start_t, end_t, num):
    front_len = int((degree + 1) / 2)
    back_len = front_len if degree % 2 == 1 else (front_len + 1)
    return np.hstack([[start_t]* front_len, np.linspace(start_t, end_t, num), [end_t]* back_len])