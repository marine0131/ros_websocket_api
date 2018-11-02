#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from xunjian_nav.msg import Encoder, Sonar


class FakeDataPub():
    def __init__(self):
        rospy.Subscriber('cmd_vel', Twist, self.cmdvel_callback)
        laser_pub = rospy.Publisher('scan', LaserScan, queue_size=1)
        enc_pub = rospy.Publisher('encoder', Encoder, queue_size=1)
        # pose_pub = rospy.Publisher('pose', PoseStamped, queue_size=1)
        sonar_pub = rospy.Publisher('range_dist', Sonar, queue_size=1)

        self.enc_msg = Encoder()
        scan_msg = LaserScan()
        sonar_msg = Sonar()

        self.enc_msg.vx = 0
        self.enc_msg.w = 0

        scan_msg.angle_increment = 0.00581718236208
        scan_msg.angle_max = 2.35572338104
        scan_msg.angle_min = -2.35619449615
        scan_msg.range_max = 10.0
        scan_msg.range_min = 0.05
        scan_msg.scan_time = 0.06666666
        scan_msg.header.frame_id = 'laser'
        with open('/home/whj/catkin_ws/src/api/api/data/scan2.txt') as f:
            scan_msg.ranges = map(float, f.readline().split(','))
            scan_msg.intensities = map(float, f.readline().split(','))

        # pose_msg.header.frame_id = 'base_link'
        # pose_msg.pose.position.x = 3.1
        # pose_msg.pose.position.y = -0.1
        # pose_msg.pose.position.z = 0
        # pose_msg.pose.orientation.w = 1.0
        # pose_msg.pose.orientation.x = 0
        # pose_msg.pose.orientation.y = 0
        # pose_msg.pose.orientation.z = 0

        sonar_msg.ranges = [20, 30, 40, 50, 60, 70]

        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            # pose_msg.pose.position.x += 0.01
            # pose_msg.pose.position.y += 0.01
            # pose_msg.pose.orientation.z += 0.01
            # pose_msg.pose.orientation.w = 1 - pose_msg.pose.orientation.z**2
            # if pose_msg.pose.position.x > 6.7 or pose_msg.pose.position.y > 9.0:
            #     pose_msg.pose.position.x = 3.0
            #     pose_msg.pose.position.y = -3.5
            # if pose_msg.pose.orientation.w < 0.6:
            #     pose_msg.pose.orientation.z = 0.01
            #enc_msg.vx = 0
            #enc_msg.w = 200
            scan_msg.header.stamp = rospy.Time.now()

            sonar_msg.ranges[0] = sonar_msg.ranges[0] + 0.1 if sonar_msg.ranges[0] < 80 else 20
            sonar_msg.ranges[1] = sonar_msg.ranges[1] + 0.1 if sonar_msg.ranges[1] < 80 else 20
            sonar_msg.ranges[2] = sonar_msg.ranges[2] + 0.1 if sonar_msg.ranges[2] < 80 else 20
            sonar_msg.ranges[3] = sonar_msg.ranges[3] + 0.1 if sonar_msg.ranges[3] < 80 else 20
            sonar_msg.ranges[4] = sonar_msg.ranges[4] + 0.1 if sonar_msg.ranges[4] < 80 else 20
            sonar_msg.ranges[5] = sonar_msg.ranges[5] + 0.1 if sonar_msg.ranges[5] < 80 else 20


            enc_pub.publish(self.enc_msg)
            laser_pub.publish(scan_msg)
            sonar_pub.publish(sonar_msg)
            rate.sleep()

    def cmdvel_callback(self, msg):
        rospy.loginfo('recieve cmd_vel: vx=%f,w=%f', msg.linear.x, msg.angular.z)
        self.enc_msg.vx = msg.linear.x*1000
        self.enc_msg.w = msg.angular.z*1000


if __name__ == '__main__':
    rospy.init_node('fake_data_pub')
    FakeDataPub()
