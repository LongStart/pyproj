import numpy as np
from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import operator

# derivative
def Derivative(p, t):
    assert(len(t) == len(p))
    d2t = t[2:] - t[:-2]
    dp = p[1:] - p[:-1]
    dt = t[1:] - t[:-1]
    v = 1 / d2t * (dp[1:]*dt[:-1]/dt[1:] + dp[:-1]*dt[1:]/dt[:-1])
    return v

def Derivative3d(txyz):
    dx_dt = Derivative(txyz[1], txyz[0])
    dy_dt = Derivative(txyz[2], txyz[0])
    dz_dt = Derivative(txyz[3], txyz[0])
    return np.array([txyz[0][1:-1], dx_dt, dy_dt, dz_dt])

# integral
def Integral3d(txyz):
    x = np.cumsum(txyz[0]*txyz[1])
    y = np.cumsum(txyz[0]*txyz[2])
    z = np.cumsum(txyz[0]*txyz[3])
    return np.array([txyz[0], x, y, z])

# moving average
def MovingAverage(p, t, half_width=3):
    assert(len(t) == len(p))
    width = 2*half_width + 1
    dt = t[1:] - t[0:-1]
    pp = (p[1:] + p[0:-1])/2
    pt = pp*dt
    p_sum = np.convolve(pt, np.ones(width),'valid')
    t_sum = np.convolve(dt, np.ones(width),'valid')
    return (p_sum / t_sum)

def MovingAverage3d(txyz, half_width=3):
    x = MovingAverage(txyz[1], txyz[0], half_width)
    y = MovingAverage(txyz[2], txyz[0], half_width)
    z = MovingAverage(txyz[3], txyz[0], half_width)
    t_begin = half_width
    t_end = len(txyz[0]) - half_width -1
    return np.array([txyz[0][t_begin:t_end], x, y, z])

# interpolation
## 3d linear
def Interpolate3d(txyz, t):
    print(type(t[2]))
    fx = interpolate.interp1d(txyz[0], txyz[1], fill_value="extrapolate")
    fy = interpolate.interp1d(txyz[0], txyz[2], fill_value="extrapolate")
    fz = interpolate.interp1d(txyz[0], txyz[3], fill_value="extrapolate")
    x = fx(t)
    y = fy(t)
    z = fz(t)
    return np.array([t, x, y, z])

## rotation
def InterpolateRotation(rot_txyz, t):
    rots = R.from_rotvec(rot_txyz[1:].transpose())
    slerp = Slerp(rot_txyz[0], rots)
    interp_rots = slerp(t)
    interp_r = np.array([rotvec.as_rotvec() for rotvec in interp_rots]).transpose()
    return np.array([t, interp_r[0], interp_r[1], interp_r[2]])

# unaligned operation
## normal operation
def UnalignedOperate3d(txyz_0, txyz_1, ope):
    t = np.array(txyz_0[0])
    txyz_1_interp = Interpolate3d(txyz_1, t)
    x = ope(txyz_0[1], txyz_1_interp[1])
    y = ope(txyz_0[2], txyz_1_interp[2])
    z = ope(txyz_0[3], txyz_1_interp[3])
    return np.array([t, x, y, z])

## rot
def UnalignedRotate(txyz, rot_txyz):
    t = txyz[0]
    rot_txyz_interp = InterpolateRotation(rot_txyz, t)
    rots = R.from_rotvec(rot_txyz_interp[1:].transpose())
    rotated = np.array(txyz)
    for i in range(len(t)):
        rotated[1:,i] = rots[i].as_dcm().dot( rotated[1:,i] )
    return rotated

if __name__ == '__main__':
    txyz = np.array([[1403715518, 1403715518, 1403715518],[1,1,2],[2,2,1],[3,3,1]])
    t = np.array([1403715518.0897748, 1403715518.099735, 1403715518.1097167])
    print(Interpolate3d(txyz, t))