from scipy.spatial.transform import Rotation as R
import numpy as np
from math import *

def delta_axis_angle(r0, r1):
    rotvec = (r0.inv()*r1).as_rotvec()
    angle = (rotvec.dot(rotvec))**0.5
    axis = rotvec / angle
    return (angle, axis)

def calc_delta_rotvec(r0, r1):
    return (r1*r0.inv()).as_rotvec()

if __name__ == '__main__':
    print('start')
    increase = pi / 18
    # rot_axis = np.array([0,0,1])
    steps = 5
    ori = R.from_quat([0,0,0,1])
    # update = R.from_rotvec(rot_axis * increase)
    pre_ori = ori
    for i in range(steps):
        this_rotvec = np.random.random(3)
        update = R.from_rotvec(this_rotvec)
        ori = update * ori
        delta_rotvec = calc_delta_rotvec(pre_ori, ori)

        this_angle = this_rotvec.dot(this_rotvec)**0.5
        this_axis = this_rotvec / this_angle
        delta_angle = delta_rotvec.dot(delta_rotvec)**0.5
        delta_axis = delta_rotvec / delta_angle
        print('this: {},{}, delta: {},{}, try axis: {}'.format(this_angle, this_axis, delta_angle, delta_axis, pre_ori.as_dcm().dot(delta_axis)))
        pre_ori = ori
    