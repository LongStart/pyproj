import math
from math import cos
from math import sin


# def quarttoeuler(w, x, y, z):
#     # yaw = math.atan2(2 * (w * x + y * z), (1 - 2 * (x * x + y * y)))
#     # pitch = math.asin(2 * (w * y - z * x))
#     # roll = math.atan2(2 * (w * z + x * y), (1 - 2 * (z * z + y * y)))

#     yaw = math.atan2(2 * (x * y - w * z), w * w + x * x - y * y - z * z)
#     pitch = math.asin(-2 * (w * y + z * x))
#     roll = math.atan2(2 * (y * z - w * x), w * w - x * x - y * y + z * z)

#     return (yaw, pitch, roll)

# def quarttoeuler(q0, q1, q2, q3):

#     yaw = math.atan2(2 * (q1 * q2 - q0 * q3), q0 *
#                      q0 + q1 * q1 - q2 * q2 - q3 * q3)
#     pitch = math.asin(-2 * (q0 * q2 + q3 * q1))
#     roll = math.atan2(2 * (q2 * q3 - q0 * q1), q0 *
#                       q0 - q1 * q1 - q2 * q2 + q3 * q3)

#     return (yaw, pitch, roll)

def quarttoeuler(q0, q1, q2, q3):

    yaw = math.atan2(2 * (q0 * q1 + q2 * q3), q0 *
                     q0 - q1 * q1 - q2 * q2 + q3 * q3)
    pitch = math.asin(2 * (q0 * q2 - q3 * q1))
    roll = math.atan2(2 * (q0 * q3 + q2 * q1), q0 *
                      q0 + q1 * q1 - q2 * q2 - q3 * q3)

    return (yaw, pitch, roll)


# def eulertoquart(yaw, pitch, roll):
#     cy = cos(yaw / 2)
#     cp = cos(pitch / 2)
#     cr = cos(roll / 2)

#     sy = sin(yaw / 2)
#     sp = sin(pitch / 2)
#     sr = sin(roll / 2)

#     # w = cy * cp * cr + sy * sp * sr
#     # x = cy * sp * cr + sy * cp * sr
#     # y = cy * cp * sr - sy * sp * cr
#     # z = sy * cp * cr - cy * sp * sr

#     w = cy * cp * cr - sy * sp * sr
#     x = cy * cp * sr + sy * sp * cr
#     y = cy * sp * cr - sy * cp * sr
#     z = sy * cp * cr + cy * sp * sr
#     return (w, x, y, z)

def eulertoquart(yaw, pitch, roll):
    cy = cos(yaw / 2)
    cp = cos(pitch / 2)
    cr = cos(roll / 2)

    sy = sin(yaw / 2)
    sp = sin(pitch / 2)
    sr = sin(roll / 2)

    q0 = cy * cp * cr - sy * sp * sr
    q1 = sy * cp * cr - cy * sp * sr
    q2 = cy * sp * cr + sy * cp * sr
    q3 = cy * cp * sr - sy * sp * cr

    return (q0, q1, q2, q3)


def quarttoeuler_deg(w, x, y, z):
    (yaw, pitch, roll) = quarttoeuler(w, x, y, z)

    return (yaw * 180 / math.pi, pitch * 180 / math.pi, roll * 180 / math.pi)


def eulertoquart_deg(yaw, pitch, roll):
    yaw = yaw / 180 * math.pi
    pitch = pitch / 180 * math.pi
    roll = roll / 180 * math.pi
    return eulertoquart(yaw, pitch, roll)


# dd = quarttoeuler(1, 0, 0, 0)
# print(dd)

alti = (0, 90, 0)

q = eulertoquart_deg(alti[0], alti[1], alti[2])

# (w, x, y, z) = eulertoquart_deg(45, 0, 0)
# print(quarttoeuler_deg(q[0], q[1], q[2], q[3]))
print(quarttoeuler_deg(q[0], q[2], q[3], q[1]))
