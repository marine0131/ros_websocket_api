#! /usr/bin/env python

import rospy
from PIL import Image
import base64
import os
import shutil as sh
import re
from api_msgs.srv import DelFile, DelFileResponse, RenameMap, RenameMapResponse, SaveModifiedMap, SaveModifiedMapResponse
from api_msgs.srv import MapData, MapDataResponse, GetList, GetListResponse


class StaticMapSrv():
    def __init__(self, pkg_path):
        self.pkg_path = pkg_path
        rospy.Service('api/get_static_map', MapData, self.handle_staticmap)
        rospy.Service('api/map_list', GetList, self.handle_maplist)
        rospy.Service('api/del_map', DelFile, self.handle_delmap)
        rospy.Service('api/rename_map', RenameMap, self.handle_renamemap)
        rospy.Service('api/save_modified_map', SaveModifiedMap, self.handle_savemodifiedmap)

    def handle_staticmap(self, req):
        full_name = os.path.join(self.pkg_path, 'maps/' + req.name + '.png')
        f = open(full_name, 'rb')
        base_data = MapDataResponse()
        base_data.mapWidth, base_data.mapHeight = Image.open(full_name).size
        base_data.base64Data = str(base64.b64encode(f.read()))

        return base_data

    def handle_maplist(self, req):
        res = GetListResponse()
        f_list = os.listdir(os.path.join(self.pkg_path, 'maps/'))
        map_list = []
        for f in f_list:
            if os.path.splitext(f)[1] == '.yaml':
                map_list.append(os.path.splitext(f)[0])

        res.list = ';'.join(map_list)
        res.msg = 'success'
        return res

    def handle_delmap(self, req):
        res = DelFileResponse()
        rospy.loginfo('request to delte map "%s"', req.name)
        try:
            os.remove(os.path.join(self.pkg_path, 'maps/' + req.name + '.yaml'))
            os.remove(os.path.join(self.pkg_path, 'maps/' + req.name + '.png'))
            os.remove(os.path.join(self.pkg_path, 'goals/' + req.name + '.json'))
            res.msg = 'success'
        except Exception as e:
            res.msg = str(e)
        return res

    def handle_renamemap(self, req):
        res = RenameMapResponse()
        rospy.loginfo('request to rename map "%s" to "%s"',
                      req.oldMapName, req.newMapName)
        yaml = os.path.join(self.pkg_path, 'maps/' + req.oldMapName + '.yaml')
        png = os.path.join(self.pkg_path, 'maps/' + req.oldMapName + '.png')
        goals = os.path.join(self.pkg_path, 'goals/' + req.oldMapName + '.json')
        if not os.path.isfile(yaml):
            res.msg = 'map not exist'
            return res
        new_yaml = os.path.join(self.pkg_path, 'maps/' + req.newMapName + '.yaml')
        new_png = os.path.join(self.pkg_path, 'maps/' + req.newMapName + '.png')
        new_goals = os.path.join(self.pkg_path, 'goals/' + req.newMapName + '.json')

        if os.path.isfile(new_yaml):
            res.msg = 'new name exist'
            return res

        try:
            os.rename(yaml, new_yaml)
            os.rename(png, new_png)
            os.rename(goals, new_goals)
        except Exception as e:
            res.msg = str(e)
            return res

        self.modify_yaml(new_yaml, req.oldMapName, req.newMapName)
        res.msg = 'success'
        return res

    def handle_savemodifiedmap(self, req):
        res = SaveModifiedMapResponse()
        old_map_file = os.path.join(self.pkg_path, 'maps/' + req.oldName + '.png')

        if not os.path.exists(old_map_file):
            res.msg = 'map not exists'
            return res

        img_data = base64.b64decode(req.base64Data.split(',')[1])
        new_map_file = os.path.join(self.pkg_path, 'maps/' + req.newName + '.png')
        try:
            with open(new_map_file, 'wb') as f:
                f.write(img_data)
        except Exception as e:
            res.msg = str(e)
            return res

        if req.oldName == req.newName:
            res.msg = 'success'
            return res

        yaml = os.path.join(self.pkg_path, 'maps/' + req.oldName + '.yaml')
        # png = os.path.join(self.pkg_path, 'maps/' + req.oldName + '.png')
        goals = os.path.join(self.pkg_path, 'goals/' + req.oldName + '.json')
        new_yaml = os.path.join(self.pkg_path, 'maps/' + req.newName + '.yaml')
        new_goals = os.path.join(self.pkg_path, 'goals/' + req.newName + '.json')

        sh.copyfile(yaml, new_yaml)
        # sh.copyfile(png, new_png)
        sh.copyfile(goals, new_goals)
        self.modify_yaml(new_yaml, req.oldName, req.newName)
        res.msg = 'success'
        return res

    def modify_yaml(self, yaml_file, new_name, old_name):
        with open(yaml_file, 'r') as f:
            lines = f.readlines()
        lines[0] = re.sub(new_name, old_name, lines[0])
        with open(yaml_file, 'w+') as f:
            f.writelines(lines)


if __name__ == "__main__":
    rospy.init_node('static_map_server')
    pkg_path = rospy.get_param('~api_pkg_path', '/home/whj/catkin_ws/src/api/api/')
    StaticMapSrv(pkg_path)
    rospy.spin()
