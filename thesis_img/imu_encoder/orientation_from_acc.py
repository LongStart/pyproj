import matplotlib.pyplot as plt
from math import *
x1 = []
y1 = []

imu_t     = []
imu_yaw   = []
imu_pitch = []
imu_roll  = []

sen_t = []
sen_gx = []
sen_gy = []
sen_gz = []
sen_ax = []
sen_ay = []
sen_az = []
sen_gr = []

# acc_yaw = []
acc_pitch = []
acc_roll = []

gyro_yaw   = []
gyro_pitch = []
gyro_roll  = []

imu_file = 'build/imu.txt'  
gyro_acc_file = 'build/gyro_acc_adc.txt'  

with open(imu_file) as fp:  
    for line in fp:
        (tt, yaw, pitch, roll) = line.split()
        tt    = float(tt   )
        yaw   = float(yaw  )* pi / 180
        pitch = float(pitch)* pi / 180
        roll  = float(roll )* pi / 180

        imu_t     += [tt   ]
        imu_yaw   += [yaw  ]
        imu_pitch += [pitch]
        imu_roll  += [roll ]

gyro_yaw   += [imu_yaw   [0]]
gyro_pitch += [imu_pitch [0]]
gyro_roll  += [imu_roll  [0]]

with open(gyro_acc_file) as fp:  
    for line in fp:
        (tt, gx,gy,gz, ax,ay,az ) = line.split()
        tt = float(tt)
        gx = float(gx)
        gy = float(gy)
        gz = float(gz)
        ax = float(ax)
        ay = float(ay)
        az = float(az)

        sen_t  += [tt]
        sen_gx += [gx]
        sen_gy += [gy]
        sen_gz += [gz]
        sen_ax += [ax]
        sen_ay += [ay]
        sen_az += [az]

        acc_pitch += [atan2(ax, (ay**2 + az**2)**0.5)/50.]
        acc_roll  += [atan2(ay, (ax**2 + az**2)**0.5)/50.]

        if(len(sen_t) > 1):
            gyro_yaw   += [ gyro_yaw  [-1] - (sen_t[-1] - sen_t[-2]) * gz/15]
            gyro_pitch += [ gyro_pitch[-1] - (sen_t[-1] - sen_t[-2]) * gy/15]
            gyro_roll  += [ gyro_roll [-1] + (sen_t[-1] - sen_t[-2]) * gx/15]


# plt.plot(sen_t, acc_pitch, '-')
# plt.plot(imu_t, imu_pitch, '-')

# plt.plot(sen_t, acc_roll, '-')
# plt.plot(imu_t, imu_roll, '-')

# plt.plot(sen_t, gyro_yaw, '-', label='gyro_yaw')
# plt.plot(imu_t, imu_yaw, '-' ,label='imu_yaw')

# plt.plot(sen_t, gyro_pitch, '-', label='gyro_pitch')
# plt.plot(imu_t, imu_pitch, '-' ,label='imu_pitch')

# plt.plot(sen_t, gyro_roll, '-', label='gyro_roll')
# plt.plot(imu_t, imu_roll, '-' ,label='imu_roll')

# plt.legend(loc='upper right', fontsize='small', ncol=1)
# plt.ylabel('俯仰角(°)')
# plt.xlabel('Unix时间戳(s)')
# plt.title('姿态传感器对比输出曲线')
# plt.grid(True)
# plt.legend(('acc_pitch','acc_roll', 'imu_pitch', 'imu_roll'))
# plt.legend(('acc_roll', 'imu_roll'))
# plt.xlim(1544073813.35, 1544073819.9)
# plt.ylim(-58, -18)
# plt.show()
# plt.savefig('encoder_imu_compare.pdf')

############################################################
# fig, axs = plt.subplots(nrows=2)

# fig.subplots_adjust(hspace=0.4)
# ax = axs[0]
# ax.plot(sen_t, acc_pitch, 'b', label='加速度计直接换算输出')
# ax.plot(imu_t, imu_pitch, 'r', label='Mahony滤波器输出')
# ax.set_ylabel('俯仰角(rad)')
# ax.set_xlabel('Unix时间戳(s)')
# ax.grid(1)
# ax.legend(loc='upper right', fontsize='small', ncol=1)
# ax = axs[1]
# ax.plot(sen_t, acc_roll, 'b')
# ax.plot(imu_t, imu_roll, 'r')
# ax.set_ylabel('横滚角(rad)')
# ax.set_xlabel('Unix时间戳(s)')
# ax.grid(1)
# plt.show()

############################################################
fig, axs = plt.subplots(nrows=3)

fig.subplots_adjust(hspace=0.4)
ax = axs[0]
ax.plot(sen_t, gyro_pitch, 'b', label='角速度积分输出')
ax.plot(imu_t, imu_pitch, 'r', label='Mahony滤波器输出')
ax.set_ylabel('俯仰角(rad)')
# ax.set_xlabel('Unix时间戳(s)')
ax.grid(1)
ax.legend(loc='lower left', fontsize='small', ncol=1)

ax = axs[1]
ax.plot(sen_t, gyro_roll, 'b', label='角速度积分输出')
ax.plot(imu_t, imu_roll, 'r', label='Mahony滤波器输出')
ax.set_ylabel('横滚角(rad)')
# ax.set_xlabel('Unix时间戳(s)')
# ax.legend(loc='lower right', fontsize='small', ncol=1)
ax.grid(1)

ax = axs[2]
ax.plot(sen_t, gyro_yaw, 'b', label='角速度积分输出')
ax.plot(imu_t, imu_yaw, 'r', label='Mahony滤波器输出')
ax.set_ylabel('偏航角(rad)')
ax.set_xlabel('Unix时间戳(s)')
# ax.legend(loc='lower right', fontsize='small', ncol=1)
ax.grid(1)
plt.show()