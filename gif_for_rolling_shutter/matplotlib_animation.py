import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots()
xdata, ydata = [0, 10], [0, 10.]

ln, = plt.plot([], [], '-', linewidth=10)


def init():
    ax.set_xlim(0, 10.)
    ax.set_ylim(0, 10.)
    return ln,

def update(frame):
    ydata = [frame, frame]
    # xdata.append(frame)
    # ydata.append(np.sin(frame))
    ln.set_data(xdata, ydata)
    return ln,

ani = FuncAnimation(fig, update, frames=np.hstack(5 * [np.linspace(0,10., 50)]) ,
                    init_func=init, blit=True, interval = 0)
ani.save('basic_animation.mp4', fps=60, extra_args=['-vcodec', 'libx264'])
plt.show()