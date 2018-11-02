#! /usr/bin/env python

import rospy
import json
import os
import math
from geometry_msgs.msg import PoseStamped
from tf import transformations
from move_base_msgs.msg import MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from api_msgs.srv import NavToGoal, NavToGoalResponse, AddGoal, AddGoalResponse, DelGoal, DelGoalResponse
from api_msgs.srv import RenameGoal, RenameGoalResponse, GoalList, GoalListResponse
from api_msgs.srv import Initialize, InitializeResponse


class GoalListSrv():
    def __init__(self, path):
        self.goal_path = path
        self.mapinfo = OccupancyGrid()
        self.pose_msg = PoseStamped()
        self.SPECIAL_GOAL = ['__CHARGE', '__INIT']  # special goal key

        rospy.Service('api/add_goal', AddGoal, self.handle_addgoal)
        rospy.Service('api/add_custom_goal', Initialize, self.handle_addcustomgoal)
        rospy.Service('api/del_goal', DelGoal, self.handle_delgoal)
        rospy.Service('api/goal_list', GoalList, self.handle_goallist)
        rospy.Service('api/init_goal_list', GoalList, self.handle_initgoallist)
        rospy.Service('api/rename_goal', RenameGoal, self.handle_renamegoal)
        rospy.Service('api/nav_to_goal', NavToGoal, self.handle_navgoal)
        rospy.Service('api/set_charge_point', AddGoal, self.handle_setcharge)
        rospy.Service('api/set_init_point', AddGoal, self.handle_setinit)

        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber('pose', PoseStamped, self.pose_callback)

        self.goal_pub = rospy.Publisher('goal', MoveBaseGoal, queue_size=1)
        # rospy.Subscriber('map_metadata', MapMetaData, self.pose_callback)

    def pose_callback(self, msg):
        self.pose_msg = msg

    def map_callback(self, msg):
        self.mapinfo.info = msg.info

    # nav to a selected goal
    def handle_navgoal(self, req):
        rospy.loginfo('nav to goal %s in map %s', req.goalName, req.mapName)
        goal_msg = MoveBaseGoal()
        res = NavToGoalResponse()

        file_name = self.goal_path + req.mapName + '.json'
        try:
            with open(file_name, 'r') as f:
                goal_dict = json.load(f)
        except IOError:
            rospy.logerr('fail to open file')

        goal_msg.target_pose.header.frame_id = 'map'
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        if req.goalName in goal_dict:
            goal_msg.target_pose.pose.orientation.x = goal_dict[req.goalName]['orientation']['x']
            goal_msg.target_pose.pose.orientation.y = goal_dict[req.goalName]['orientation']['y']
            goal_msg.target_pose.pose.orientation.z = goal_dict[req.goalName]['orientation']['z']
            goal_msg.target_pose.pose.orientation.w = goal_dict[req.goalName]['orientation']['w']
            goal_msg.target_pose.pose.position.x = goal_dict[req.goalName]['position']['x']
            goal_msg.target_pose.pose.position.y = goal_dict[req.goalName]['position']['y']
            goal_msg.target_pose.pose.position.z = goal_dict[req.goalName]['position']['z']
            res.msg = 'success'
        else:
            rospy.logwarn('goal does not exist')
            res.msg = 'goal not exist'
            return res
        # rospy.loginfo(goal_msg)
        self.goal_pub.publish(goal_msg)
        return res

    # reanme goal
    def handle_renamegoal(self, req):
        rospy.loginfo('rename goal "%s" to "%s" in map "%s"', req.mapName,
                      req.oldGoalName, req.newGoalName)
        res = RenameGoalResponse()
        json_file = os.path.join(self.goal_path, req.mapName + '.json')
        goal_dict = {}
        try:
            with open(json_file, 'r') as f:
                goal_dict = json.load(f)
        except Exception:
            res.msg = 'can not open json file'
            return res
        try:
            goal_dict[req.newGoalName] = goal_dict.pop(req.oldGoalName)
        except Exception:
            res.msg = 'old name not exist or new name exist'
            return res

        # if rename a goal with special function, rename the special key too
        for tag in self.SPECIAL_GOAL:
            if goal_dict[tag] == req.oldGoalName:
                goal_dict[tag] = req.newGoalName

        with open(json_file, 'w') as f:
            json.dump(goal_dict, f)
        res.msg = 'success'
        return res

    # add current pose as a goal
    def handle_addgoal(self, req):
        rospy.loginfo('add a goal "%s" in map "%s"', req.goalName, req.mapName)
        res = AddGoalResponse()
        json_path = self.goal_path + req.mapName + '.json'
        res.msg = self.save_to_json(json_path, req.goalName, self.get_posedict(self.pose_msg))
        return res

    # add custom pose as a goal
    def handle_addcustomgoal(self, req):
        rospy.loginfo('add a goal "%s" in map "%s"', req.goalName, req.mapName)
        res = InitializeResponse()
        json_path = self.goal_path + req.mapName + '.json'

        # get custom pose
        pose = PoseStamped()
        pose.pose.position.x = req.pose.x * self.mapinfo.info.resolution + self.mapinfo.info.origin.position.x
        pose.pose.position.y = req.pose.y * self.mapinfo.info.resolution + self.mapinfo.info.origin.position.y
        head_x = req.head.x * self.mapinfo.info.resolution + self.mapinfo.info.origin.position.x
        head_y = req.head.y * self.mapinfo.info.resolution + self.mapinfo.info.origin.position.y
        yaw = math.atan2(head_y - pose.pose.position.y, head_x - pose.pose.position.x)
        quat = transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        res.msg = self.save_to_json(json_path, req.goalName, self.get_posedict(pose))
        return res

    # delete goal
    def handle_delgoal(self, req):
        rospy.loginfo('delete goal "%s" in map "%s"', req.goalName, req.mapName)
        res = DelGoalResponse()
        file_name = self.goal_path + req.mapName + '.json'

        if not os.path.exists(file_name):
            res.msg = 'goal list not exit'
            return res

        with open(file_name, 'r') as load_f:
            goal_dict = json.load(load_f)

        # unable to delete goal with special function
        for tag in self.SPECIAL_GOAL:
            if goal_dict[tag] == req.goalName:
                res.msg = 'can not delete this goal'
                return res

        try:
            goal_dict.pop(req.goalName)
        except Exception as e:
            res.msg = str(e)
            return res

        with open(file_name, 'w') as f:
            json.dump(goal_dict, f)
        res.msg = 'success'
        return res

    # get goal list
    def handle_goallist(self, req):
        res = GoalListResponse()
        goal_file = self.goal_path + req.mapName + '.json'

        # if goal list file no exist, initialize the goal list
        if not os.path.exists(goal_file):
            self.init_goallist(goal_file)

        try:
            with open(goal_file, 'r') as f:
                goal_dict = json.load(f)
        except Exception as e:
            res.msg = str(e)
            return res

        # get special tags
        tags = {}
        for tag in self.SPECIAL_GOAL:
            tags[tag] = goal_dict.pop(tag)

        # get goal list
        goal_grid_dict = dict((name, goal_dict[name]['grid']) for name in goal_dict)

        res.list = json.dumps(goal_grid_dict)
        res.tags = json.dumps(tags)
        res.msg = 'success'
        return res

    # init goallist
    def handle_initgoallist(self, req):
        res = GoalListResponse()
        goal_path = os.path.join(self.goal_path, req.mapName + '.json')
        if os.path.exists(goal_path):
            os.remove(goal_path)
        self.init_goallist(goal_path)
        res.msg = 'success'
        return res

    def handle_setcharge(self, req):
        res = AddGoalResponse()
        file_path = self.goal_path + req.mapName + '.json'
        res.msg = self.set_special_goal(file_path, self.SPECIAL_GOAL[0], req.goalName)
        return res

    def handle_setinit(self, req):
        res = AddGoalResponse()
        file_path = self.goal_path + req.mapName + '.json'
        res.msg = self.set_special_goal(file_path, self.SPECIAL_GOAL[1], req.goalName)
        return res

    # set a goal with special function
    def set_special_goal(self, file_path, type_name, goal_name):
        try:
            with open(file_path, 'r') as f:
                goal_dict = json.load(f)
        except Exception as e:
            msg = str(e)
            return msg

        if goal_dict.get(goal_name):  # goal name exist
            goal_dict[type_name] = goal_name
            with open(file_path, 'w') as f:
                json.dump(goal_dict, f)
        else:
            msg = 'can not find goal name'
            return msg

        msg = 'success'
        return msg

    # init a map's goal list
    def init_goallist(self, goal_path):
        # initial pose
        pose = PoseStamped()
        pose.pose.orientation.w = 1.0
        init_pose = self.get_posedict(pose)
        goal_dict = {}
        goal_dict['__CHARGE'] = 'charge'
        goal_dict['__INIT'] = 'init'
        goal_dict['charge'] = init_pose
        goal_dict['init'] = init_pose
        # save to json
        with open(goal_path, 'w') as f:
            json.dump(goal_dict, f)

    # get a custom dict of pose
    def get_posedict(self, pose_msg):
        pose_dict = {}
        quat_dict = {}
        grid_dict = {}

        pose = {}
        pose_dict['x'] = pose_msg.pose.position.x
        pose_dict['y'] = pose_msg.pose.position.y
        pose_dict['z'] = pose_msg.pose.position.z
        quat_dict['x'] = pose_msg.pose.orientation.x
        quat_dict['y'] = pose_msg.pose.orientation.y
        quat_dict['z'] = pose_msg.pose.orientation.z
        quat_dict['w'] = pose_msg.pose.orientation.w
        grid_dict['x'] = (int)((pose_msg.pose.position.x
                               - self.mapinfo.info.origin.position.x)
                               / self.mapinfo.info.resolution)
        grid_dict['y'] = (int)((pose_msg.pose.position.y
                               - self.mapinfo.info.origin.position.y)
                               / self.mapinfo.info.resolution)
        grid_dict['z'] = 0

        pose['position'] = pose_dict
        pose['orientation'] = quat_dict
        pose['grid'] = grid_dict

        return pose

    # read the json file, add a goal and save to json
    def save_to_json(self, path, goal_name, pose):
        goal_dict = {}
        if not os.path.exists(path):
            self.init_goallist(path)

        with open(path, 'r') as load_f:
            goal_dict = json.load(load_f)

        if not goal_dict.get(goal_name):
            goal_dict[goal_name] = pose
        else:
            msg = 'ducplicated goal name'
            return msg

        with open(path, 'w') as f:
            json.dump(goal_dict, f)
        msg = 'success'
        return msg


if __name__ == "__main__":
    rospy.init_node('goal_server')
    pkg_path = rospy.get_param('~api_pkg_path', '/home/whj/catkin_ws/src/api/api/')
    goal_path = pkg_path + 'goals/'
    GoalListSrv(goal_path)
    rospy.spin()
