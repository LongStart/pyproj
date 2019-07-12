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

    mag_angle_rate = [SignalXd.from_t_vals(sig.t, Magnitude(sig.xyz)) for sig in raw_imus_angle_rate]

    resample_t = np.linspace(raw_imus_angle_rate[0].t[0], raw_imus_angle_rate[0].t[-1], len(raw_imus_angle_rate[0].t))
    evensample_mag = [SignalXd(Interpolate(sig.t_vals, resample_t)) for sig in mag_angle_rate]

    corr_t = np.arange(-len(resample_t) + 1, len(resample_t))
    corr = np.correlate(evensample_mag[0].vals[0], evensample_mag[1].vals[0], mode='full')
    # print(corr)
    corr = SignalXd.from_t_vals(corr_t, corr)
    dt = corr.t[np.argmax(corr.vals[0])] * (resample_t[1] - resample_t[0])
    print(np.argmax(corr.vals[0]))
    print(dt)
    # quit()
    evensample_mag[0].t = evensample_mag[0].t - dt
    raw_imus_angle_rate[0].t = raw_imus_angle_rate[0].t - dt

    angle_rate = {
        'angle_rate_p20': raw_imus_angle_rate[0].t_xyz,
        'angle_rate_s9': raw_imus_angle_rate[1].t_xyz}

    angle_rate_y = {
        'angle_rate_p20': np.vstack((raw_imus_angle_rate[0].t, raw_imus_angle_rate[0].xyz[1])) ,
        'angle_rate_s9': np.vstack((raw_imus_angle_rate[1].t, raw_imus_angle_rate[1].xyz[1]))}

    mag_angle_rate = {
            'mag_angle_rate_p20': evensample_mag[0].t_vals,
            'mag_angle_rate_s9': evensample_mag[1].t_vals}

    correlation = {
            'corr': corr.t_vals}

    plotter = PlotCollection.PlotCollection("Multiple Wave")
    add_naxis_figure(plotter, "angle rate", angle_rate, linewidth=0.8, fmt='.-')
    add_naxis_figure(plotter, "mag angle rate", mag_angle_rate, linewidth=0.8, fmt='-')
    add_naxis_figure(plotter, "correlation", correlation, linewidth=0.8, fmt='.-')
    add_naxis_figure(plotter, "angle_rate_y", angle_rate_y, markersize=35, fmt='_')
    
    plotter.show()

    