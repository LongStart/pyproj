import numpy as np
from scipy.spatial.transform import Rotation as R
from lie_algebra import hat

def Derivative3d(t, xyz):
    dt = np.gradient(t)
    dxyz = np.gradient(xyz, axis=1)
    dx_dt = dxyz[0]/dt
    dy_dt = dxyz[1]/dt
    dz_dt = dxyz[2]/dt
    return np.array([dx_dt, dy_dt, dz_dt])

def AngleRate(t, xyzw):
    dt = np.gradient(t)
    r = R.from_quat(xyzw.transpose())
    dr = np.zeros((len(r), 3))
    dr[1:-1] = (r[:-2].inv() * r[2:]).as_rotvec() * 0.5
    dr[0] = (r[0].inv() * r[1]).as_rotvec()
    dr[-1] = (r[-2].inv() * r[-1]).as_rotvec()

    dx_dt = dr.transpose()[0] / dt
    dy_dt = dr.transpose()[1] / dt
    dz_dt = dr.transpose()[2] / dt
    
    return np.array([dx_dt, dy_dt, dz_dt])

def ToRotationMat(xyzw):
    return R.from_quat(xyzw.transpose()).as_dcm()

def AngleRateInBody(t, xyzw):
    angle_rate = AngleRate(t, xyzw)
    rot_inv = R.from_quat(xyzw.transpose()).inv().as_dcm()
    result = np.empty((np.shape(xyzw)[1], 4))
    for i in range(len(result)):
        result[i,0] = t[i]
        result[i,1:] = rot_inv[i].dot(angle_rate.transpose()[i])
    return result.transpose()

def StaticRotVec3d(s_xyzw, xyz):
    rot_mat = R.from_quat(s_xyzw).as_dcm()
    return np.array([rot_mat.dot(v) for v in xyz.transpose()]).transpose()