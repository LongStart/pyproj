import rospy
from geometry_msgs.msg import Vector3

pub = rospy.Publisher('euler_puber', Vector3, queue_size=10)
rospy.init_node('imu_msg_converter', anonymous=True)
rate = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    euler_angle = Vector3()
    euler_angle.x = 0
    euler_angle.y = 1
    euler_angle.z = 2

    pub.publish(euler_angle)
    rate.sleep()