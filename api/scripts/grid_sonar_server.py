#! /usr/bin/env python

import rospy
import json
import numpy as np
from tf import TransformListener
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from xunjian_nav.msg import Sonar
from api_msgs.srv import GridSonar, GridSonarResponse


class GridSonarSrv():
    def __init__(self, SONAR_FRAME, MAP_FRAME, ROBOT_FRAME):
        # GOAL_LIST_DIR = rospy.get_param('~goal_list_dir')
        # self.path = MAP_DIR
        self.sonar_frame = SONAR_FRAME
        self.map_frame = MAP_FRAME
        self.robot_frame = ROBOT_FRAME
        self.mapinfo = OccupancyGrid()
        self.pose_msg = PoseStamped()
        self.sonar_msg = []

        self.tf_listener = TransformListener()
        rospy.Service('api/grid_sonar', GridSonar, self.handle_gridsonar)
        rospy.Service('api/rt_grid_sonar', GridSonar, self.handle_realtimesonar)
        rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber('range_dist', Sonar, self.sonar_callback, queue_size=1)
        rospy.Subscriber('pose', PoseStamped, self.pose_callback, queue_size=1)

    def pose_callback(self, msg):
        self.pose_msg = msg

    def handle_realtimesonar(self, req):
        i = 0
        sonar = {}
        rot = self.get_rotate_matrix()[0:3]

        for t in self.sonar_frame:
            pt = {}
            sonar_pt = PointStamped()
            sonar_pt.header.frame_id = t
            # transform sonar center point from sonar to base_link
            # self.tf_listener.waitForTransform(self.map_frame, t, rospy.Time.now(),
            #                                   rospy.Duration(1.0))
            temp = self.tf_listener.transformPoint(self.robot_frame, sonar_pt)

            # calc sonar center coordinate in map
            # p = np.dot([temp.point.x, temp.point.y, 0], rot)
            # p = [p[0]+self.pose_msg.pose.position.x, p[1]+self.pose_msg.pose.position.y, 0]
            p = [temp.point.x * rot[0][0] + temp.point.y * rot[1][0] + self.pose_msg.pose.position.x,
                 temp.point.x * rot[0][1] + temp.point.y * rot[1][1] + self.pose_msg.pose.position.y]

            # calc sonar center grid in map
            start_x = (int)((p[0] - self.mapinfo.info.origin.position.x)
                            / self.mapinfo.info.resolution)
            start_y = (int)((p[1] - self.mapinfo.info.origin.position.y)
                            / self.mapinfo.info.resolution)

            # calc sonar end point from sonar to base_link
            sonar_pt.point.x = self.sonar_msg[i] / 100.0
            i += 1
            temp = self.tf_listener.transformPoint(self.robot_frame, sonar_pt)
            # calc sonar end coordinate in map
            # p = np.dot([temp.point.x, temp.point.y, 0], rot)
            # p = [p[0]+self.pose_msg.pose.position.x, p[1]+self.pose_msg.pose.position.y, 0]
            p = [temp.point.x * rot[0][0] + temp.point.y * rot[1][0] + self.pose_msg.pose.position.x,
                 temp.point.x * rot[0][1] + temp.point.y * rot[1][1] + self.pose_msg.pose.position.y]

            # calc sonar end grid in map
            end_x = (int)((p[0] - self.mapinfo.info.origin.position.x)
                          / self.mapinfo.info.resolution)
            end_y = (int)((p[1] - self.mapinfo.info.origin.position.y)
                          / self.mapinfo.info.resolution)

            pt['start'] = [start_x, start_y]
            pt['end'] = [end_x, end_y]
            sonar[t] = pt

        mapInfo = {}
        mapInfo['gridWidth'] = self.mapinfo.info.width
        mapInfo['gridHeight'] = self.mapinfo.info.height
        mapInfo['resolution'] = self.mapinfo.info.resolution

        res = GridSonarResponse()
        res.gridPoint = json.dumps(sonar)
        res.mapInfo = json.dumps(mapInfo)
        res.msg = 'success'
        return res

    def get_rotate_matrix(self):
        quat = (-self.pose_msg.pose.orientation.x, -self.pose_msg.pose.orientation.y,
                -self.pose_msg.pose.orientation.z, self.pose_msg.pose.orientation.w)
        return self.tf_listener.fromTranslationRotation(translation=(0, 0, 0),
                                                        rotation=quat)

    def handle_gridsonar(self, req):
        i = 0
        sonar = {}
        for t in self.sonar_frame:
            pt = {}
            sonar_pt = PointStamped()
            sonar_pt.header.frame_id = t
            # transform sonar center point from sonar to base_link
            self.tf_listener.waitForTransform(self.robot_frame, t, rospy.Time.now(),
                                              rospy.Duration(1.0))
            temp = self.tf_listener.transformPoint(self.robot_frame, sonar_pt)
            # calc sonar center grid in map
            # start_x = (int)((temp.point.x - self.mapinfo.info.origin.position.x)
            #                 / self.mapinfo.info.resolution)
            # start_y = (int)((temp.point.y - self.mapinfo.info.origin.position.y)
            #                 / self.mapinfo.info.resolution)
            start_x = (int)(temp.point.x / 0.01)
            start_y = (int)(temp.point.y / 0.01)

            # calc sonar end point from sonar to base_link
            sonar_pt.point.x = self.sonar_msg[i] / 100.0
            i += 1
            temp = self.tf_listener.transformPoint(self.robot_frame, sonar_pt)
            # calc sonar end grid in map
            # end_x = (int)((temp.point.x - self.mapinfo.info.origin.position.x)
            #               / self.mapinfo.info.resolution)
            # end_y = (int)((temp.point.y - self.mapinfo.info.origin.position.y)
            #               / self.mapinfo.info.resolution)
            end_x = (int)(temp.point.x / 0.01)
            end_y = (int)(temp.point.y / 0.01)

            pt['start'] = [start_x, start_y]
            pt['end'] = [end_x, end_y]
            pt['range'] = sonar_pt.point.x
            sonar[t] = pt

        mapInfo = {}
        # mapInfo['gridWidth'] = self.mapinfo.info.width
        # mapInfo['gridHeight'] = self.mapinfo.info.height
        # mapInfo['resolution'] = self.mapinfo.info.resolution

        res = GridSonarResponse()
        res.gridPoint = json.dumps(sonar)
        res.mapInfo = json.dumps(mapInfo)
        res.msg = 'success'
        return res

    def map_callback(self, msg):
        self.mapinfo.info = msg.info

    def sonar_callback(self, msg):
        self.sonar_msg = map(int, msg.ranges)


if __name__ == "__main__":
    rospy.init_node('grid_sonar_server')
    sonar_frame = rospy.get_param('~sonar_frame').split(',')
    map_frame = rospy.get_param('~map_frame', 'map')
    robot_frame = rospy.get_param('~robot_frame', 'base_link')
    GridSonarSrv(sonar_frame, map_frame, robot_frame)
    rospy.spin()
