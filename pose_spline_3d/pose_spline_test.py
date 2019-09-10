import sm
import bsplines
import numpy as np
from scipy.spatial.transform import Rotation as R

if __name__ == "__main__":
    bsp = bsplines.BSplinePose(4, sm.RotationVector())
    curve = np.array([[0.1,0.2,0.3, 0.4,0.5,0.6], [0.1,0,0, 0,0,0.1], [0.2,0,0, 0,0,0.1]]).T
    bsp.initPoseSplineSparse(np.array([0.,1,2]), curve, 5, 1e-5)
    r0 = R.from_dcm(bsp.orientation(0.0)).as_rotvec()
    r0_ = R.from_dcm(bsp.curveValueToTransformation(curve.T[0])[:3,:3]).as_rotvec()
    t0_ = bsp.curveValueToTransformation(curve.T[0])[:3,3]
    print(r0)
    print(t0_)

