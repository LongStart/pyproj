import numpy as np
from scipy.spatial.transform import Rotation as R

def f(rotvec, a):
    return R.from_rotvec(a * rotvec)

if __name__ == "__main__":
    print("666666")
    rotvec = np.array([1,2,3])
    