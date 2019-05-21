import matplotlib.pyplot as plt
import matplotlib
from math import *
from simple_system import SimpleSystem
from pid_controller import PIDController
from signal_generator import PWMGenerator


font_name = "Microsoft YaHei"
matplotlib.rcParams['font.family']=font_name
matplotlib.rcParams['axes.unicode_minus']=False # in case minus sign is shown as box

t = []
y = []
x = []
t_step_len = 1e-2
sim_time = 5.
system = SimpleSystem(t_step_len)
controller = PIDController(200, 50, 500, 0.05, (-500,500))
# pwm = PWMGenerator(t_step_len, 2, .5)

ts = 0.
desire_val = 0
yaw_stop_time = 2.5

while ts < sim_time:
    t += [ts]
    y += [system.observe()]
    x += [desire_val]

    observe = 0
    sample_time = 50
    for i in range(0,sample_time):
        observe += system.observe()
    observe /= float(sample_time)
    control = controller.update(observe, desire_val)
    system.update(control)
    if(ts < yaw_stop_time):
        desire_val += t_step_len*20
    # desire_val = -3.878 + 15 * pwm.update()
    # desire_val = 10 + 0 * pwm.update()
    
    ts += t_step_len



plt.plot(t, y, '-')
plt.plot(t, x, '--')
plt.ylabel('偏航角 (°)')
plt.xlabel('时间 (s)')
plt.title('偏航角控制-斜坡响应曲线')
plt.grid(True)
plt.legend(('观测值','目标值'))
# plt.show()
plt.savefig('yaw_control.tif',dpi=600)