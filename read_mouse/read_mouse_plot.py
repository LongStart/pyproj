import matplotlib.pyplot as plt
from time import sleep
from pynput.mouse import Button, Controller
pos_x = []
pos_y = []
ts = []
mouse = Controller()
for i in range(0,400):
    pos = mouse.position
    pos_x += [pos[0]]
    pos_y += [pos[1]]
    ts += [i]
    plt.clf()
    # plt.plot(pos_x, pos_y)
    plt.plot(ts, pos_y,'.-')
    plt.pause(1e-3)

print(pos_y)
plt.show()

