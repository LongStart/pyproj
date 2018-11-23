from math import *

def euler_to_quaternion(yaw, pitch, roll):
    yaw = yaw /180. * pi
    pitch = pitch /180. * pi
    roll = roll /180. * pi

    yaw *= -1.
    pitch *= -1.

    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)

    w = cy * cr * cp + sy * sr * sp
    x = cy * sr * cp - sy * cr * sp
    y = cy * cr * sp + sy * sr * cp
    z = sy * cr * cp - cy * sr * sp

    return (x,y,z,w)
    