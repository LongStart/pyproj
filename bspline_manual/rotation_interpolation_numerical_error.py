import numpy as np
from scipy.spatial.transform import Rotation as R

def f(rotvec, a):
    return R.from_rotvec(a * rotvec)

if __name__ == "__main__":
    print("666666")
    r = R.from_rotvec([[1,0,0],[0,0,1]])

    t = 0.5

    interp_r_a = R.from_rotvec(r[0].as_rotvec() * (1 - t)) * R.from_rotvec(r[1].as_rotvec() * t)
    interp_r_b = r[0] * R.from_rotvec(t * (r[0].inv() * r[1]).as_rotvec())

    print(interp_r_a.as_rotvec())
    print(interp_r_b.as_rotvec())
    