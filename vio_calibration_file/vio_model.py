import numpy as np

class IMUModel():
    def __init__(self, an_aw_gn_gw):
        (self.acc_n, self.acc_w, self.gyr_n, self.gyr_w) = an_aw_gn_gw

class PinHoleRollingShutterCameraModel():
    def __init__(self, intrinsic, resolution_w_h, distortion, t_rolling_shutter):
        (self.fx, self.fy, self.cx, self.cy) = intrinsic
        (self.k1, self.k2, self.p1, self.p2, self.k3) = distortion
        self.rolling_shutter_time = t_rolling_shutter
        self.width, self.height = resolution_w_h

    @property
    def projection_mat(self):
        k = np.array([[fx, 0, cx],[0, fy, cy],[0, 0, 1]])
        return k

class CameraIMUExtrinsicModel():
    def __init__(self, tf_cam_to_imu, td_cam_to_imu):
        assert(np.shape(tf_cam_to_imu) == (4,4))
        self.transform_cam_to_imu = tf_cam_to_imu
        self.td_cam_to_imu = td_cam_to_imu

    @classmethod
    def InitByImuToCam(cls, tf_imu_to_cam, td_imu_to_cam):
        result = cls(tf_imu_to_cam, td_imu_to_cam)
        result.td_cam_to_imu = result.td_imu_to_cam
        result.transform_cam_to_imu = result.transform_imu_to_cam
        return result

    @property
    def rotation_cam_to_imu(self):
        return self.transform_cam_to_imu[:3, :3]

    @property
    def rotation_imu_to_cam(self):
        return self.rotation_cam_to_imu.transpose()
    
    @property
    def translation_cam_to_imu(self):
        return self.transform_cam_to_imu[:3, 3]

    @property
    def translation_imu_to_cam(self):
        return -self.rotation_cam_to_imu.transpose().dot(self.translation_cam_to_imu)

    @property
    def transform_imu_to_cam(self):
        tf = np.identity(4)
        tf[:3,:3] = self.rotation_imu_to_cam
        tf[:3, 3] = self.translation_cam_to_imu
        return tf
    
    @property
    def td_imu_to_cam(self):
        return -self.td_cam_to_imu

    def as_dict(self):
        import inspect
        return dict(inspect.getmembers(self))

class VIOModel():
    def __init__(self, \
        imu_an_aw_gn_gw=[0.]*4, \
        cam_intrin=[0.]*4, resolution=[0]*2, cam_distor=[0.]*5, t_rolling_shutter=0., \
        tf_cam_to_imu=np.zeros((4,4)), td_cam_to_imu=0., \
        imu_topic='/imu0', cam_topic='/cam0'):
        self.imu = IMUModel(imu_an_aw_gn_gw)
        self.cam = PinHoleRollingShutterCameraModel(cam_intrin, resolution, cam_distor, t_rolling_shutter)
        self.cam_imu_ex = CameraIMUExtrinsicModel(tf_cam_to_imu, td_cam_to_imu)
        self.imu_topic = imu_topic
        self.cam_topic = cam_topic
    
    def as_dict(self):
        from copy import deepcopy
        result = deepcopy(vars(self))
        result['cam_imu_ex'] = self.cam_imu_ex.as_dict()
        result['imu'] = deepcopy(vars(self.imu))
        result['cam'] = deepcopy(vars(self.cam))
        return result