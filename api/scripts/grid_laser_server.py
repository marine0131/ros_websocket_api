#!/usr/bin/env python
import rospy
import math
# import numpy as np
import json
from tf import TransformListener
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from api_msgs.msg import GridPoint
from api_msgs.srv import GridLaser, GridLaserResponse


class GridLaserSrv():

    def __init__(self, map_frame, robot_frame):
        self.map_frame = map_frame
        self.robot_frame = robot_frame

        self.pose_msg = PoseStamped()
        self.mapinfo = OccupancyGrid()
        self.laser_data = LaserScan()

        self.tf_listener = TransformListener()
        rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('scan', LaserScan, self.laser_callback)
        rospy.Service('api/rt_grid_laser', GridLaser, self.handle_realtimelaser)
        rospy.Service('api/grid_laser', GridLaser, self.handle_gridlaser)

    def pose_callback(self, msg):
        self.pose_msg = msg

    def handle_realtimelaser(self, req):
        laser_point = PointStamped()
        laser_point.header.frame_id = self.laser_data.header.frame_id
        res = GridLaserResponse()
        index = 0
        self.tf_listener.waitForTransform(self.robot_frame, laser_point.header.frame_id,
                                          rospy.Time(), rospy.Duration(1.0))

        rot = self.get_rotate_matrix()[0:3]

        # now = rospy.Time.now().to_sec()
        for r in self.laser_data.ranges:
            # laser point coordinate in laser frame
            angle = self.laser_data.angle_min + self.laser_data.angle_increment * index
            laser_point.point.x = r * math.cos(angle)
            laser_point.point.y = r * math.sin(angle)
            index = index + 1

            # calc laser point coodinate in map frame
            temp = self.tf_listener.transformPoint(self.robot_frame, laser_point)
            # calc laser point coordinate in map frame
            # temp.point.x = self.pose_msg.pose.position.x + temp.point.x
            # temp.point.y = self.pose_msg.pose.position.y + temp.point.y
            p = [temp.point.x * rot[0][0] + temp.point.y * rot[1][0] + self.pose_msg.pose.position.x,
                 temp.point.x * rot[0][1] + temp.point.y * rot[1][1] + self.pose_msg.pose.position.y]
            # p = np.dot([temp.point.x, temp.point.y, 0], rot)
            # p = [p[0]+self.pose_msg.pose.position.x, p[1]+self.pose_msg.pose.position.y, 0]

            # calc grid point coordinate in grid map
            pp = GridPoint()
            pp.x = (int)((p[0] - self.mapinfo.info.origin.position.x)
                         / self.mapinfo.info.resolution)
            pp.y = (int)((p[1] - self.mapinfo.info.origin.position.y)
                         / self.mapinfo.info.resolution)

            res.gridPoint.append(pp)

        # rospy.logwarn(rospy.Time.now().to_sec() - now)
        mapInfo = {}
        mapInfo['gridWidth'] = self.mapinfo.info.width
        mapInfo['gridHeight'] = self.mapinfo.info.height
        mapInfo['resolution'] = self.mapinfo.info.resolution
        res.mapInfo = json.dumps(mapInfo)
        res.msg = 'success'

        return res

    def get_rotate_matrix(self):
        quat = (-self.pose_msg.pose.orientation.x, -self.pose_msg.pose.orientation.y,
                -self.pose_msg.pose.orientation.z, self.pose_msg.pose.orientation.w)
        return (self.tf_listener.fromTranslationRotation(translation=(0, 0, 0), rotation=quat))

    def handle_gridlaser(self, req):
        laser_point = PointStamped()
        laser_point.header.frame_id = self.laser_data.header.frame_id
        res = GridLaserResponse()
        index = 0
        for r in self.laser_data.ranges:
            # laser point coordinate in laser frame
            laser_point.point.x = r * math.cos(self.laser_data.angle_min
                                               + self.laser_data.angle_increment * index)
            laser_point.point.y = r * math.sin(self.laser_data.angle_min
                                               + self.laser_data.angle_increment * index)
            index = index + 1

            # calc laser point coodinate in robot frame
            temp = self.tf_listener.transformPoint(self.robot_frame, laser_point)

            # calc laser point coordinate in vertual grid map
            pp = GridPoint()
            # pp.x = (int)((temp.point.x - self.mapinfo.info.origin.position.x)
            #              / self.mapinfo.info.resolution)
            # pp.y = (int)((temp.point.y - self.mapinfo.info.origin.position.y)
            #              / self.mapinfo.info.resolution)

            pp.x = (int)(temp.point.x / 0.01)
            pp.y = (int)(temp.point.y / 0.01)
            res.gridPoint.append(pp)

        mapInfo = {}
        # mapInfo['gridWidth'] = self.mapinfo.info.width
        # mapInfo['gridHeight'] = self.mapinfo.info.height
        # mapInfo['resolution'] = self.mapinfo.info.resolution
        res.mapInfo = json.dumps(mapInfo)
        res.msg = 'success'

        return res

    # map callback, update map data,actually, map data need no update
    def map_callback(self, msg):
        self.mapinfo.info = msg.info

    # laser callback, update laser data
    def laser_callback(self, msg):
        self.laser_data.header.frame_id = msg.header.frame_id
        self.laser_data.angle_increment = msg.angle_increment
        self.laser_data.angle_max = msg.angle_max
        self.laser_data.angle_min = msg.angle_min
        self.laser_data.ranges = msg.ranges


if __name__ == '__main__':
    rospy.init_node('gird_laser_server')
    map_frame = rospy.get_param('~map_frame', 'map')
    robot_frame = rospy.get_param('~robot_frame', 'base_link')
    GridLaserSrv(map_frame, robot_frame)
    rospy.spin()
