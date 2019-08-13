import socket, traceback
import rospy
from sensor_msgs.msg import Imu

def InitSocket(host='', port=5555):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.bind((host, port))
    return s

def InitRosPub():
    pub = rospy.Publisher('/Android/Imu', Imu, queue_size=100)
    rospy.init_node('android_forwarder', anonymous=True)
    return pub

def UdpMsgToRosMsg(udp_msg):
    data_list = [float(v) for v in udp_msg.split(',')]
    if(len(data_list) < 9): 
        return None
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.from_sec(data_list[0])
    imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = data_list[2:5]
    imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = data_list[6:9]
    return imu_msg

def RunForwarder(s, pub):
    
    while 1:
        try:
            message, address = s.recvfrom(8192)
            # print (message)
            imu_msg = UdpMsgToRosMsg(message)
            if imu_msg is not None:
                pub.publish(imu_msg)
        except (KeyboardInterrupt, SystemExit):
            raise   
        except:
            traceback.print_exc()
            quit()

if __name__ == "__main__":
    pub = InitRosPub()
    sock = InitSocket(host='', port=5555)
    RunForwarder(sock, pub)