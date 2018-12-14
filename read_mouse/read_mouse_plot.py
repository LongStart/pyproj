import matplotlib.pyplot as plt
from time import sleep
from pynput.mouse import Button, Controller
pos_x = []
pos_y = []
ts = []
mouse = Controller()
for i in range(0,2000):
    pos = mouse.position
    pos_x += [pos[0]]
    pos_y += [pos[1]]
    ts += [i]
    sleep(1e-3)
    # plt.clf()
    # plt.plot(ts, pos_y,'.-')
    # plt.pause(1e-3)

print(pos_y)
plt.plot(ts, pos_y,'.-')
plt.show()

