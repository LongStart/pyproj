import numpy as np
from scipy.spatial.transform import Rotation as R

def f(rotvec, a):
    return R.from_rotvec(a * rotvec)

if __name__ == "__main__":
    print("666666")
    # r = R.from_rotvec([[1,0,0],[0,0,1]])
    r = R.from_rotvec([[1,0,0],[1,0,0]])

    t = 3

    interp_r_a = R.from_rotvec(r[0].as_rotvec() * (1 - t)) * R.from_rotvec(r[1].as_rotvec() * t)
    interp_r_aa = r[0] * R.from_rotvec(r[0].as_rotvec() * -t) * R.from_rotvec(r[1].as_rotvec() * t)
    interp_r_b = r[0] * R.from_rotvec(t * (r[0].inv() * r[1]).as_rotvec())

    # print(interp_r_a.as_rotvec())
    # print(interp_r_aa.as_rotvec())
    # print(interp_r_b.as_rotvec())

    # right_aa = R.from_rotvec(r[0].as_rotvec() * -t) * R.from_rotvec(r[1].as_rotvec() * t)
    # right_b = R.from_rotvec(t * (r[0].inv() * r[1]).as_rotvec())

    # print(right_aa.as_rotvec())
    # print(right_b.as_rotvec())

    a = R.from_rotvec(r[0].as_rotvec() * t) * R.from_rotvec(r[1].as_rotvec() * t)
    b = R.from_rotvec((r[0] * r[1]).as_rotvec() * t)
    print(a.as_rotvec())
    print(b.as_rotvec())
    