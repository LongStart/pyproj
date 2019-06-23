from scipy.spatial.transform import Rotation as R
import numpy as np
from math import *

if __name__ == '__main__':
    print('start')
    a = pi / 2
    rot_axis_0 = np.array([1, 0, 0])  # rotate around x axis
    rot_axis_1 = np.array([0, 1, 0])  # rotate around y axis

    orient_0 = R.from_rotvec(rot_axis_0 * a)
    orient_1 = R.from_rotvec(rot_axis_1 * a)

    delta_0_1 = (orient_0.inv() * orient_1).as_rotvec()
    delta_1_0 = (orient_1.inv() * orient_0).as_rotvec()
    print(delta_0_1)
    print(delta_1_0)
    # print((pi - acos(delta[3])) * 2)
    # print(delta.dot(delta)**0.5)