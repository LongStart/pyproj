import numpy as np
from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.signal import medfilt
from scipy.signal import butter
from scipy.signal import filtfilt
from scipy.ndimage.filters import uniform_filter1d
import operator
from bisect import bisect
###### All the input and output arguments should be np.ndarray

# time const 
def TimeConstantVector3d(t, xyz):
    if isinstance(t, int) or isinstance(t, float):
        return np.array([[t], [xyz[0]], [xyz[1]], [xyz[2]]])    
    x = np.array([xyz[0]] * len(t))
    y = np.array([xyz[1]] * len(t))
    z = np.array([xyz[2]] * len(t))
    return np.array([t, x, y, z])

# derivative
def Derivative(p, t):
    assert(len(t) == len(p))
    assert(len(t) > 2)
    d2t = t[2:] - t[:-2]
    dp = p[1:] - p[:-1]
    dt = t[1:] - t[:-1]
    v = 1 / d2t * (dp[1:]*dt[:-1]/dt[1:] + dp[:-1]*dt[1:]/dt[:-1])
    return v

def Derivative3d(txyz):
    if 1 == len(txyz.transpose()):
        return np.array([txyz[0][:], [0], [0], [0]])    
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

def weight_by_interval(p, t):
    assert(len(t) == len(p))
    assert(len(t) > 2)
    dt = t[1:] - t[0:-1]
    pp = (p[1:] + p[0:-1])/2
    pt = pp*dt
    p2t = pt[0:-1] + pt[1:]
    d2t = t[2:] - t[:-2]
    return p2t/d2t

# rotation
def Rotate(txyz, rot_vec):
    r = R.from_rotvec(rot_vec)
    result = np.array(txyz)
    for i in range(len(result.transpose())):
        result[1:,i] = r.apply(result[1:,i]) 
    return result 

# moving average
def MovingAverage(p, t, kernel_size):
    weighted_p = weight_by_interval(p, t)
    return uniform_filter1d(weighted_p, kernel_size)

def MovingAverage3d(txyz, kernel_size):
    if 1 == len(txyz.transpose()):
        return np.array(txyz)    
    x = MovingAverage(txyz[1], txyz[0], kernel_size)
    y = MovingAverage(txyz[2], txyz[0], kernel_size)
    z = MovingAverage(txyz[3], txyz[0], kernel_size)
    return np.array([txyz[0][1:-1], x, y, z])

# medfilter
def Medfilter(p, t, kernel_size=5):
    weighted_p = weight_by_interval(p, t)
    return medfilt(weighted_p, kernel_size=kernel_size)

def Medfilter3d(txyz, kernel_size=5):
    if 1 == len(txyz.transpose()):
        return np.array(txyz)
    x = Medfilter(txyz[1], txyz[0], kernel_size)
    y = Medfilter(txyz[2], txyz[0], kernel_size)
    z = Medfilter(txyz[3], txyz[0], kernel_size)
    return np.array([txyz[0][1:-1], x, y, z])

# low pass
def LFilter(p, t, order, critical_freq):
    b,a = butter(order, critical_freq)
    weighted_p = weight_by_interval(p, t)
    return filtfilt(b, a, weighted_p)

def LFilter3d(txyz, order, critical_freq):
    if 1 == len(txyz.transpose()):
        return np.array(txyz)
    x = LFilter(txyz[1], txyz[0], order, critical_freq)
    y = LFilter(txyz[2], txyz[0], order, critical_freq)
    z = LFilter(txyz[3], txyz[0], order, critical_freq)
    return np.array([txyz[0][1:-1], x, y, z])


# interpolation
## 3d linear
def Interpolate3d(txyz, t):
    if 1 == len(txyz.transpose()):
        return TimeConstantVector3d(t, txyz.transpose()[1:])
    fx = interpolate.interp1d(txyz[0], txyz[1], fill_value="extrapolate")
    fy = interpolate.interp1d(txyz[0], txyz[2], fill_value="extrapolate")
    fz = interpolate.interp1d(txyz[0], txyz[3], fill_value="extrapolate")
    return np.array([t, fx(t), fy(t), fz(t)])

## rotation
def InterpolateRotation(rot_txyz, t):
    if 1 == len(rot_txyz.transpose()):
        return TimeConstantVector3d(t, rot_txyz.transpose()[1:])
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
def RotateByRotVec(txyz, rot_vec):
    r = R.from_rotvec(rot_vec)
    rotated = np.array(txyz)
    for i in range(len(txyz.transpose())):
        rotated[1:,i] = r.apply( rotated[1:,i] )
    return rotated

def UnalignedRotate(txyz, rot_txyz):
    if(len(rot_txyz.transpose()) == 1):
        return RotateByRotVec(txyz, rot_txyz[1:].transpose())

    i_begin = bisect(txyz[0], rot_txyz[0,0])
    i_end = bisect(txyz[0], rot_txyz[0,-1])
    result_txyz = np.array(txyz[:, i_begin: i_end])
    rot_txyz_interp = InterpolateRotation(rot_txyz, result_txyz[0])
    
    for i in range(len(result_txyz[0])):
        result_txyz[1:,i] = R.from_rotvec(rot_txyz_interp[1:,i]).apply( result_txyz[1:,i] )
    return result_txyz


if __name__ == '__main__':
    txyz = np.array([[1403715518, 1403715518, 1403715518],[1,1,2],[2,2,1],[3,3,1]])
    t = np.array([1403715518.0897748, 1403715518.099735, 1403715518.1097167])
    print(Interpolate3d(txyz, t))