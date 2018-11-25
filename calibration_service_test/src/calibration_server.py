#!/usr/bin/env python
NAME = 'calibration_server.py'

from calibration_service_test.srv import *
# import service_test
import rospy
# import time

# print(service_test_add_int)

def handle_calibration(req):
    return CalibrationResponse(55)

def calibration_server_init():
    rospy.init_node('calibration_server')
    s = rospy.Service('calibrate_imu', Calibration, handle_calibration)
    # print "Ready to add two ints."
    # rospy.spin()

if __name__ == "__main__":
    # add_two_ints_server()
    calibration_server_init()
    # while True:
    #     pass
    rospy.spin()
