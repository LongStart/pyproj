import numpy as np
import matplotlib.pyplot as plt
if __name__ == "__main__":
    print('awesome!')
    rows = 2
    init_x = np.array([0.] * rows)
    x = np.array(init_x)
    y = np.linspace(0, rows - 1, rows)
    max_x = 10.
    
    # plt.show()
    
    while True:
        plt.clf()
        x += 1.
        if x[0] > max_x:
            x = np.array(init_x)
        plt.plot(x, y, '-', linewidth=15)
        plt.xlim(0, 10)
        plt.pause(1e-8)