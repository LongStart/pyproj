import rospy
import rosbag
import numpy as np
from sys import argv
from vicon_correction import CorrectBiasedStamp
from scipy.spatial.transform import Rotation as R
from scipy.ndimage.filters import uniform_filter1d
from ros_io import *
from inertia_from_traj import *
# from signal import Trajectory3d
import PlotCollection
from add_3axis_figure import *
from dsp import *
import scipy.interpolate as interpolate

if __name__ == '__main__':
    if(len(argv) < 3):
        print("Example: python acc_from_traj.py dataset_name bag_path ")
        quit()
    dataset_name = argv[1]
    bag_filename = argv[2]
    
    groundtruth_topic_name = ''
    imu_topic_name = ''
    if dataset_name == 'euroc':
        groundtruth_topic_name = '/vicon/firefly_sbx/firefly_sbx'
        imu_topic_name = '/imu0'
        groundtruth_frame_name = "world"
        body_frame_name = "vicon/firefly_sbx/firefly_sbx"
    else:
        print('unknown dataset')
        quit()

    q_vicon_to_imu = R.from_dcm([[0.33638, -0.01749,  0.94156],[-0.02078, -0.99972, -0.01114],[0.94150, -0.01582, -0.33665]]).as_quat()
    t_vicon_to_imu = np.array([0.06901, -0.02781, -0.12395])
    vicon_to_imu_xyz_xyzw = np.hstack((t_vicon_to_imu, q_vicon_to_imu))
    imu_to_vicon_xyz_xyzw = np.hstack((-1 * t_vicon_to_imu, -1 * q_vicon_to_imu))
    gravity = np.array([0., 0., 9.81])

    transform_msgs = ReadTopicMsg(bag_filename, groundtruth_topic_name)
    imu_msgs = ReadTopicMsg(bag_filename, imu_topic_name)

    raw_gt_pose = PoseFromTransformStamped(transform_msgs)
    CorrectBiasedStamp(raw_gt_pose[0])

    # t_0 = raw_gt_pose[0].mean()
    # t = [t_0 - 1, t_0, t_0 + 1]
    # k = 3
    # t = np.r_[(raw_gt_pose[0,0],)*(k+1),
    #       t,
    #       (raw_gt_pose[0,-1],)*(k+1)]
    # print(t)

    spl_param_x = interpolate.splrep(raw_gt_pose[0], raw_gt_pose[1], s = 0.01, k=4)
    spl_param_y = interpolate.splrep(raw_gt_pose[0], raw_gt_pose[2], s = 0.01, k=4)
    spl_param_z = interpolate.splrep(raw_gt_pose[0], raw_gt_pose[3], s = 0.01, k=4)
    spl_x = interpolate.BSpline(*spl_param_x, extrapolate=False)
    spl_y = interpolate.BSpline(*spl_param_y, extrapolate=False)
    spl_z = interpolate.BSpline(*spl_param_z, extrapolate=False)
    spl_txyz = np.vstack((raw_gt_pose[0], spl_x(raw_gt_pose[0]), spl_y(raw_gt_pose[0]), spl_z(raw_gt_pose[0])))

    v_txyz = np.vstack((raw_gt_pose[0], Derivative3d(raw_gt_pose[0], raw_gt_pose[1:4])))
    a_txyz = np.vstack((raw_gt_pose[0], Derivative3d(raw_gt_pose[0], v_txyz[1:4])))
    spl_v_txyz = np.vstack((raw_gt_pose[0], spl_x.derivative()(raw_gt_pose[0]), spl_y.derivative()(raw_gt_pose[0]), spl_z.derivative()(raw_gt_pose[0])))
    spl_a_txyz = np.vstack((raw_gt_pose[0], spl_x.derivative().derivative()(raw_gt_pose[0]), spl_y.derivative().derivative()(raw_gt_pose[0]), spl_z.derivative().derivative()(raw_gt_pose[0])))

    plotter = PlotCollection.PlotCollection("Multiple Wave")

    vel = {
        'by_diff': v_txyz,
        'by_spl': spl_v_txyz}
    pos = {
        'by_diff': raw_gt_pose[:4],
        'by_spl': spl_txyz
    }
    acc = {
        'by_diff': a_txyz,
        'by_spl': spl_a_txyz
    }
    # gyro = {
    #     'from_traj': imu_angle_rate,
    #     'from_imu': raw_imu_angle_rate}
    # acc = {
    #     'from_traj': imu_acc_from_traj,
    #     'from_imu': raw_imu_acc}

    # add_3axis_figure(plotter, "angle_rate", gyro, linewidth=1., fmt='-')
    # add_naxis_figure(plotter, "acc", acc, linewidth=1., fmt='-')
    add_naxis_figure(plotter, "vel", vel, linewidth=1, fmt='-')
    add_naxis_figure(plotter, "pos", pos, linewidth=1, fmt='-')
    add_naxis_figure(plotter, "acc", acc, linewidth=1, fmt='-')
    
    plotter.show()