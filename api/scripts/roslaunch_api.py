#!/usr/bin/env python
import rospy
import subprocess
import rosnode
import os
import re
from PIL import Image
from api_msgs.srv import LaunchCmd, LaunchCmdResponse, GoalList, GoalListRequest
from std_srvs.srv import Empty, EmptyRequest


class RosLaunchApi():
    def __init__(self, pkg_path):
        self.pkg_path = pkg_path
        self.current_map = None
        rospy.Service("api/remote_launch", LaunchCmd, self.handle_launch)
        self.initGoalListClient = rospy.ServiceProxy("api/init_goal_list", GoalList)

        self.reset_odom_client = rospy.ServiceProxy("reset_odom", Empty)

    def handle_launch(self, req):
        res = LaunchCmdResponse()
        reset_odom_req = EmptyRequest()
        res.msg = "fail"
        status = True
        check_node = None

        if req.cmd == "start_mapping":
            rospy.loginfo('remote run: start mapping')
            self.start_mapping()
            self.reset_odom_client(reset_odom_req)  # call reset odom service
            check_node = '/cartographer_node'

        elif req.cmd == "start_map_server":
            rospy.loginfo('remote run: start map_server')
            self.start_map_server(req.mapName)
            self.reset_odom_client(reset_odom_req)  # call reset odom service
            self.current_map = req.mapName
            check_node = '/map_server'

        elif req.cmd == "start_navigation":
            rospy.loginfo('remote run: start navigation')
            self.start_navigation()
            check_node = '/move_base'

        elif req.cmd == "stop_mapping":
            rospy.loginfo('remote run: stop mapping')
            self.stop_mapping()
            res.msg = 'success'

        elif req.cmd == "save_map":
            rospy.loginfo('remote run: save map')
            map_path = os.path.join(self.pkg_path, 'maps/' + req.mapName)
            if self.save_map(map_path):
                res.msg = 'success'
            else:
                res.msg = 'fail'

        elif req.cmd == "stop_map_server":
            rospy.loginfo('remote run: stop map_server')
            self.stop_map_server()
            res.msg = 'success'

        elif req.cmd == "stop_navigation":
            rospy.loginfo('remote run: stop navgation')
            self.stop_navgation()
            res.msg = 'success'

        elif req.cmd == "start_record":
            rospy.loginfo('remote run: start recording')
            split_size = 200  # 200 MB
            path = self.pkg_path + 'bags/'
            self.record_bag(split_size, path)
            check_node = '/record'

        elif req.cmd == "stop_record":
            rospy.loginfo('remote run: stop recording')
            self.stop_record()
            res.msg = 'success'

        else:
            res.msg = "command not found"

        if check_node:
            t0 = rospy.Time.now().to_sec()
            while(status and rospy.Time.now().to_sec() - t0 < 5.0):
                nodes = rosnode.get_node_names()
                if check_node in nodes:
                    res.msg = 'success'
                    status = False

        return res

    def start_mapping(self):
        nodes = rosnode.get_node_names()
        if '/amcl' in nodes:
            self.stop_navgation()
        if '/map_server' in nodes:
            self.stop_map_server()
        if '/cartographer_node' in nodes:
            self.stop_mapping()
            rospy.sleep(rospy.Duration(1.0))
        subprocess.Popen("roslaunch cartographer_ros my_revo_lds.launch", shell=True)

    def start_navigation(self):
        nodes = rosnode.get_node_names()
        if '/move_base' not in nodes:
            subprocess.Popen("roslaunch xunjian_nav xunjian_move_base.launch", shell=True)

    def start_map_server(self, map_name):
        cmd = "rosrun map_server map_server __name:=map_server " + self.pkg_path + "maps/" + map_name + ".yaml"
        nodes = rosnode.get_node_names()
        if '/cartographer_node' in nodes:
            self.stop_mapping()
            rospy.sleep(rospy.Duration(1.0))
        if '/map_server' in nodes:
            if map_name != self.current_map:
                self.stop_map_server()
                rospy.sleep(rospy.Duration(1.0))
                subprocess.Popen(cmd, shell=True)
        else:
            subprocess.Popen(cmd, shell=True)

    def stop_navgation(self):
        subprocess.Popen("rosnode kill amcl", shell=True)
        subprocess.Popen("rosnode kill move_base", shell=True)

    def stop_mapping(self):
        subprocess.Popen("rosnode kill cartographer_node", shell=True)
        subprocess.Popen("rosnode kill pose_publisher", shell=True)

    def stop_map_server(self):
        subprocess.Popen("rosnode kill map_server", shell=True)

    def save_map(self, map_path):
        subprocess.Popen("rosrun map_server map_saver -f " + map_path, shell=True)
        # delay for saving map
        rospy.sleep(rospy.Duration(2.0))
        # change map ext and modify yaml file
        res = self.pgm2png(map_path + '.pgm', os.path.join(self.pkg_path, 'maps/'))
        if not res:
            return res
        self.modify_yaml(map_path + '.yaml', 'pgm', 'png')

        req = GoalListRequest()
        req.mapName = map_path.split('/')[-1]
        self.initGoalListClient(req)

        return res

    def record_bag(self, split_size, path):
        subprocess.Popen("rosrun rosbag record __name:=record -a --split --size="
                         + str(split_size) + " -o " + path, shell=True)

    def stop_record(self):
        subprocess.Popen("rosnode kill record", shell=True)

    def pgm2png(self, pgm_file, png_dir):
        try:
            pgm = Image.open(pgm_file)
        except IOError:
            rospy.logerr('error: file not exit')
            return False
        name = (str)(os.path.join(png_dir, os.path.splitext(
            os.path.basename(pgm_file))[0])) + ".png"
        try:
            pgm.save(name)
            # os.remove(pgm_file)
        except Exception as e:
            rospy.logerr(str(e))
            return False
        return True

    def modify_yaml(self, yaml_file, pattern, repl):
        with open(yaml_file, 'r') as f:
            lines = f.readlines()
        lines[0] = re.sub(pattern, repl, lines[0])
        with open(yaml_file, 'w+') as f:
            f.writelines(lines)


if __name__ == "__main__":
    rospy.init_node("roslaunch_api", anonymous=True)
    path = rospy.get_param("~api_pkg_path", "/home/whj/catkin_ws/src/api/api/")
    RosLaunchApi(path)
    rospy.spin()
