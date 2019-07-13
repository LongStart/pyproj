import rospy
import rosbag
import numpy as np
from sys import argv
from vicon_correction import CorrectBiasedStamp
from scipy.spatial.transform import Rotation as R
from scipy.ndimage.filters import uniform_filter1d
from ros_io import *
from inertia_from_traj import *
from core.assignable_space_signal import *
import PlotCollection
from add_3axis_figure import *
from core.dsp import *

def GyroRotationEstimation(gyro_seq1, gyro_seq2):
    from core.least_square import Problem
    from core.least_square import gauss_newton

    p = Problem(gyro_seq1, gyro_seq2)
    return gauss_newton(p, [0.1,  0.,  0], step=20)

def CorrespondenceData(signal1, signal2):
    start1 = 0
    start2 = 0
    end1 = -1
    end2 = -1
    while signal1.t[start1] < signal2.t[0]:
        start1 += 1
    
    while signal2.t[start2] < signal1.t[0]:
        start2 += 1

    while signal1.t[end1] > signal2.t[-1]:
        end1 -= 1 

    while signal2.t[end2] > signal1.t[-1]:
        end2 -= 1 

    end1 = None if (end1 == -1) else (end1 + 1)
    end2 = None if (end2 == -1) else (end2 + 1)

    print("{}#{}####{}#{}".format(start1, start2, end1, end2))
    # print("{}#{}####{}#{}".format(signal1.t[start1], signal2.t[start2], signal1.t[end1], signal2.t[end2]))
    
    return (signal1.xyz[:,start1:end1], signal2.xyz[:, start2:end2])

if __name__ == '__main__':
    if(len(argv) < 2):
        print("Example: python dual_imu_time_diff.py bag_path ")
        quit()

    bag_filename = argv[1]
    
    imu_topic_names = ['/p20/imu0', '/s9/imu0']
    raw_imus_angle_rate = []
    for imu_topic in imu_topic_names:
        raw_imus_angle_rate.append(
            Signal3d(
                AngleRateFromIMU(
                    ReadTopicMsg(bag_filename, imu_topic))))

    mag_angle_rate = [Signal1d.from_t_x(sig.t, Magnitude(sig.xyz)) for sig in raw_imus_angle_rate]

    resample_t = np.linspace(raw_imus_angle_rate[0].t[0], raw_imus_angle_rate[0].t[-1], len(raw_imus_angle_rate[0].t))
    uniform_sample_mag = [Signal1d(Interpolate(sig.t_x, resample_t)) for sig in mag_angle_rate]
    uniform_sample_gyros = [Signal3d(Interpolate(sig.t_xyz, resample_t)) for sig in raw_imus_angle_rate]

    
    # r_1_2 = R.from_rotvec([0.13790409,  0.1652222,  -0.03514399])

    corr_t = np.arange(-len(resample_t) + 1, len(resample_t))
    corr = np.correlate(uniform_sample_mag[0].x, uniform_sample_mag[1].x, mode='full')
    # print(corr)
    corr = Signal1d.from_t_x(corr_t, corr)
    dt = corr.t[np.argmax(corr.x)] * (resample_t[1] - resample_t[0])
    print(np.argmax(corr.x))
    print(dt)
    # quit()
    uniform_sample_mag[1].t = uniform_sample_mag[1].t + dt
    raw_imus_angle_rate[1].t = raw_imus_angle_rate[1].t + dt
    uniform_sample_gyros[1].t += dt


    corresed_gyro0, corresed_gyro1 = CorrespondenceData(uniform_sample_gyros[0], uniform_sample_gyros[1])
    print("shape: {}".format(np.shape(corresed_gyro0)))
    print("shape: {}".format(np.shape(corresed_gyro1)))
    # quit()
    r_1_2 = R.from_rotvec(GyroRotationEstimation(corresed_gyro0, corresed_gyro1))

    angle_rate_rotated = Signal3d(raw_imus_angle_rate[1].t_xyz)
    angle_rate_rotated.xyz = r_1_2.apply (raw_imus_angle_rate[1].xyz.transpose()).transpose()
    # raw_imus_angle_rate[1].xyz = r_1_2.apply (raw_imus_angle_rate[1].xyz.transpose()).transpose()

    angle_rate = {
        'angle_rate_p20': raw_imus_angle_rate[0].t_xyz,
        'angle_rate_s9': raw_imus_angle_rate[1].t_xyz,
        'angle_rate_s9_roted': angle_rate_rotated.t_xyz,}

    angle_rate_y = {
        'angle_rate_p20': np.vstack((raw_imus_angle_rate[0].t, raw_imus_angle_rate[0].xyz[1])) ,
        'angle_rate_s9': np.vstack((raw_imus_angle_rate[1].t, raw_imus_angle_rate[1].xyz[1]))}

    mag_angle_rate = {
            'mag_angle_rate_p20': uniform_sample_mag[0].t_x,
            'mag_angle_rate_s9': uniform_sample_mag[1].t_x}

    correlation = {
            'corr': corr.t_x}

    plotter = PlotCollection.PlotCollection("Multiple Wave")
    add_naxis_figure(plotter, "angle rate", angle_rate, linewidth=0.5, fmt='-')
    add_naxis_figure(plotter, "mag angle rate", mag_angle_rate, linewidth=0.8, fmt='-')
    add_naxis_figure(plotter, "correlation", correlation, linewidth=0.8, fmt='.-')
    add_naxis_figure(plotter, "angle_rate_y", angle_rate_y, markersize=35, fmt='_')
    
    plotter.show()

    