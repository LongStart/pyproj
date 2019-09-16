import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from copy import deepcopy

from projection import *

class RadTanPinhole():
    def __init__(self, resolution=480, intrinsic=np.array([500., 500, 320, 240]), distortion=[0.]*5):
        pass

    def Project(self):
        pass

class PinholeLine():
    def __init__(self, position=[0,0,0.], orientation=[0,0,0], ):
        self.position = np.array(position)
        self.orientation = R.from_rotvec(orientation)
        self.intrinsic_array = intrinsic
        self.intrinsic = np.identity(3)
        self.intrinsic[0,0] = intrinsic[0]
        self.intrinsic[1,1] = intrinsic[1]
        self.intrinsic[0,2] = intrinsic[2]
        self.intrinsic[1,2] = intrinsic[3]
        self.distortion = np.array(distortion)
        self.resolution = resolution