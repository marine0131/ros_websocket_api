#!/usr/bin/env python
import rospy
import math
import os
import json
from tf import TransformListener, transformations
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseGoal
from api_msgs.srv import GridPose, GridPoseResponse, SetGoal, SetGoalResponse
from api_msgs.srv import Initialize, InitializeResponse
from api_msgs.srv import GridSonar, GridSonarResponse


class RobotPoseSrv():
    def __init__(self, map_frame, robot_frame, pkg_path):
        self.robot_frame = robot_frame
        self.map_frame = map_frame
        self.pkg_path = pkg_path
        self.pose_msg = PoseStamped()
        self.mapinfo = OccupancyGrid()
        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("pose", PoseStamped, self.pose_callback)
        # pose_pub = rospy.Publisher('grid_pose',GridPose,queue_size=10)
        rospy.Service('api/grid_pose', GridPose, self.handle_pose)
        rospy.Service('api/footprint', GridSonar, self.handle_footprint)
        rospy.Service('api/set_goal', SetGoal, self.handle_goal)
        rospy.Service('api/custom_initialize', Initialize, self.handle_custominitialize)
        rospy.Service('api/predefined_initialize', Initialize, self.handle_predefinedinitialize)

        self.custom_goal_pub = rospy.Publisher('goal', MoveBaseGoal, queue_size=10)
        self.initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.tf_listener = TransformListener()

        # rate=rospy.Rate(5)
        # while not rospy.is_shutdown():
        #     if self.mapinfo.info.resolution != 0.0:
        #         pose_grid.grid_x = (int)((p.pose.position.x-m.info.origin.position.x)/m.info.resolution)
        #         pose_grid.grid_y = (int)((p.pose.position.y-m.info.origin.position.y)/m.info.resolution)
        #         rospy.loginfo('grid pose(%d,%d)',pose_grid.grid_x,pose_grid.grid_y)
        #         pose_pub.publish(pose_grid)
        #     rate.sleep()

    def pose_callback(self, msg):
        self.pose_msg = msg

    def map_callback(self, msg):
        self.mapinfo.info = msg.info

    def handle_pose(self, req):
        grid_pose = GridPoseResponse()
        # center = PointStamped()
        # head = PointStamped()
        # center.header.frame_id = self.robot_frame
        # head.header.frame_id = self.robot_frame
        # center.point.x = 0.0
        # head.point.x = 0.4  # afront of robot
        # self.tf_listener.waitForTransform(self.map_frame, self.robot_frame,
        #                                   rospy.Time.now(), rospy.Duration(1.0))
        euler = transformations.euler_from_quaternion([self.pose_msg.pose.orientation.x,
                                                       self.pose_msg.pose.orientation.y,
                                                       self.pose_msg.pose.orientation.z,
                                                       self.pose_msg.pose.orientation.w])
        grid_center = [self.pose_msg.pose.position.x, self.pose_msg.pose.position.y]
        grid_head = [grid_center[0] + math.cos(euler[2]), grid_center[1] + math.sin(euler[2])]
        try:
            # grid_center = self.tf_listener.transformPoint(self.map_frame, center)
            # grid_head = self.tf_listener.transformPoint(self.map_frame, head)

            grid_pose.pose.x = int((grid_center[0]
                                   - self.mapinfo.info.origin.position.x)
                                   / self.mapinfo.info.resolution)
            grid_pose.pose.y = int((grid_center[1]
                                   - self.mapinfo.info.origin.position.y)
                                   / self.mapinfo.info.resolution)
            grid_pose.head.x = int((grid_head[0]
                                   - self.mapinfo.info.origin.position.x)
                                   / self.mapinfo.info.resolution)
            grid_pose.head.y = int((grid_head[1]
                                   - self.mapinfo.info.origin.position.y)
                                   / self.mapinfo.info.resolution)
            grid_pose.msg = 'success'
        except Exception:
            grid_pose.msg = 'fail to request robot pose'
        # rospy.loginfo(grid_pose)
        return grid_pose

    def handle_footprint(self, req):
        res = GridSonarResponse()
        # footprint = ast.literal_eval(rospy.get_param('/pose_server/robot_param/footprint'))
        footprint = rospy.get_param('/pose_server/robot_param/footprint')
        point = []
        for p in footprint:
            # x = (int)((p[0] - self.mapinfo.info.origin.position.x) / self.mapinfo.info.resolution)
            # y = (int)((p[1] - self.mapinfo.info.origin.position.y) / self.mapinfo.info.resolution)
            x = (int)(p[0] / 0.01)
            y = (int)(p[1] / 0.01)

            dic = {}
            dic['x'] = x
            dic['y'] = y
            point.append(dic)

        mapInfo = {}
        # mapInfo['gridWidth'] = self.mapinfo.info.width
        # mapInfo['gridHeight'] = self.mapinfo.info.height
        # mapInfo['resolution'] = self.mapinfo.info.resolution

        res.gridPoint = str(point)
        res.mapInfo = json.dumps(mapInfo)
        res.msg = 'success'
        return res

    def handle_goal(self, req):
        rospy.loginfo('recieve a custom goal: x %d, y %d', req.grid_x, req.grid_y)
        g = MoveBaseGoal()
        res = SetGoalResponse()
        g.target_pose.header.frame_id = self.map_frame
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = (req.grid_x * self.mapinfo.info.resolution
                                         + self.mapinfo.info.origin.position.x)
        g.target_pose.pose.position.y = (req.grid_y * self.mapinfo.info.resolution
                                         + self.mapinfo.info.origin.position.y)
        g.target_pose.pose.orientation.w = 1.0
        self.custom_goal_pub.publish(g)
        res.msg = 'success'
        return res

    def handle_custominitialize(self, req):
        rospy.loginfo('initalize...')
        res = InitializeResponse()
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        pose.pose.pose.position.x = req.pose.x * self.mapinfo.info.resolution + self.mapinfo.info.origin.position.x
        pose.pose.pose.position.y = req.pose.y * self.mapinfo.info.resolution + self.mapinfo.info.origin.position.y
        head_x = req.head.x * self.mapinfo.info.resolution + self.mapinfo.info.origin.position.x
        head_y = req.head.y * self.mapinfo.info.resolution + self.mapinfo.info.origin.position.y
        try:
            yaw = math.atan2(head_y - pose.pose.pose.position.y, head_x - pose.pose.pose.position.x)
            quat = transformations.quaternion_from_euler(0, 0, yaw)
            pose.pose.pose.orientation.x = quat[0]
            pose.pose.pose.orientation.y = quat[1]
            pose.pose.pose.orientation.z = quat[2]
            pose.pose.pose.orientation.w = quat[3]
            res.msg = 'success'
        except Exception:
            res.msg = 'error with calculate'
            return res

        self.initial_pose_pub.publish(pose)
        return res

    def handle_predefinedinitialize(self, req):
        rospy.loginfo('initalize...')
        res = InitializeResponse()
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        file_name = os.path.join(self.pkg_path, 'goals/' + req.mapName + '.json')
        try:
            with open(file_name, 'r') as f:
                goal_dict = json.load(f)
        except Exception:
            res.msg = 'map not exist'
            return res
        if req.goalName in goal_dict:
            pose.pose.pose.orientation.x = goal_dict[req.goalName]['orientation']['x']
            pose.pose.pose.orientation.y = goal_dict[req.goalName]['orientation']['y']
            pose.pose.pose.orientation.z = goal_dict[req.goalName]['orientation']['z']
            pose.pose.pose.orientation.w = goal_dict[req.goalName]['orientation']['w']
            pose.pose.pose.position.x = goal_dict[req.goalName]['position']['x']
            pose.pose.pose.position.y = goal_dict[req.goalName]['position']['y']
            pose.pose.pose.position.z = goal_dict[req.goalName]['position']['z']
            res.msg = 'success'
        else:
            res.msg = 'goal not exist'
            return res

        self.initial_pose_pub.publish(pose)
        return res


if __name__ == '__main__':
    rospy.init_node('pose_server', anonymous=True)
    robot_frame = rospy.get_param('~robot_frame', 'base_link')
    map_frame = rospy.get_param('~map_frame', 'map')
    pkg_path = rospy.get_param('~pkg_path', '/home/whj/catkin_ws/src/api/api/')
    RobotPoseSrv(map_frame, robot_frame, pkg_path)
    rospy.spin()
