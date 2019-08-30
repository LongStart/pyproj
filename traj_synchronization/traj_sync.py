from core.assignable_space_signal import Trajectory3d
from core.assignable_space_signal import Signal3d
from core.assignable_space_signal import Signal1d
import PlotCollection
from add_3axis_figure import *
import numpy as np
from core.dsp import *
import bsplines

def LoadTUMTrajectory(filename):
    data_matrix = []
    with open(filename, 'r') as f:
        for line in f.readlines():
            data = [float(v) for v in line.split(" ")]
            data_matrix.append(data)

    return Trajectory3d(np.array(data_matrix[50:]).T)

def CreateAngularVelocity(traj, lamb=1e-5, max_time=1000):
    order = 5
    quat_spl = bsplines.UnitQuaternionBSpline(order)
    quat_spl.initUniformSpline(traj.t, traj.xyzw, max_time, lamb)
    angular_velocity_xyz = np.array([quat_spl.getEvaluatorAt(t).evalAngularVelocity() for t in traj.t]).T
    angular_velocity = Signal3d.from_t_xyz(traj.t, angular_velocity_xyz)
    return angular_velocity

def AngularVelocityMagnitude(ang_vel):
    mag = Signal1d.from_t_x(ang_vel.t, Magnitude(ang_vel.xyz))
    return mag

if __name__ == "__main__":
    from sys import argv
    if(len(argv) < 2):
        print("Example: python {} tum_traj.txt".format(argv[0]))
        quit()

    traj_filenames = argv[1:]
    plotter = PlotCollection.PlotCollection("Multiple Wave")
    quat_for_plt = {}
    ang_vel_for_plt = {}
    ang_vel_mag_for_plt = {}
    ref_mag = None

    idx = 0
    for traj_filename in traj_filenames:
        traj = LoadTUMTrajectory(traj_filename)
        traj.xyzw = ContiguousQuaternion(traj.xyzw)

        ang_vel = CreateAngularVelocity(traj)
        ang_vel_mag = AngularVelocityMagnitude(ang_vel)

        if 0 == idx:
            ref_mag = ang_vel_mag
        else:
            dt = TimeSyncByCorrelation(ref_mag.t_x, ang_vel_mag.t_x, 10.)
            traj.t += dt
            ang_vel.t += dt
            ang_vel_mag.t += dt
            print("dt: {}".format(dt))

        quat_for_plt["quat_{}".format(idx)] = traj.t_xyzw
        ang_vel_for_plt["ang_vel_{}".format(idx)] = ang_vel.t_xyz
        ang_vel_mag_for_plt["ang_vel_mag_{}".format(idx)] = ang_vel_mag.t_x
        idx += 1

    add_naxis_figure(plotter, "quat", quat_for_plt, fmt='-')
    add_naxis_figure(plotter, "ang_vel", ang_vel_for_plt, fmt='-')
    add_naxis_figure(plotter, "ang_vel_mag", ang_vel_mag_for_plt, fmt='-')
    plotter.show()