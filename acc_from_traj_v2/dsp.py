import numpy as np
from scipy.spatial.transform import Rotation as R

def Derivative3d(t, xyz):
    dt = np.gradient(t)
    dxyz = np.gradient(xyz, axis=1)
    dx_dt = dxyz[0]/dt
    dy_dt = dxyz[1]/dt
    dz_dt = dxyz[2]/dt
    return np.array([dx_dt, dy_dt, dz_dt])

def AngleRate(t, wxyz):
    dt = np.gradient(t)
    r = R.from_quat(wxyz.transpose())
    dr = np.zeros((len(r), 3))
    dr[1:-1] = (r[:-2].inv() * r[2:]).as_rotvec() * 0.5
    dr[0] = (r[0].inv() * r[1]).as_rotvec()
    dr[-1] = (r[-2].inv() * r[-1]).as_rotvec()

    dx_dt = dr.transpose()[0] / dt
    dy_dt = dr.transpose()[1] / dt
    dz_dt = dr.transpose()[2] / dt
    
    return np.array([dx_dt, dy_dt, dz_dt])
