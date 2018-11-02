#! /usr/bin/env python

import rospy
from PIL import Image
import os.path
import re
from api.srv import MapRename, MapRenameResponse


class MapRenameSrv():
    def __init__(self, map_path):
        self.path = map_path
        srv = rospy.Service('map_rename', MapRename, self.handle_maprename)

    def handle_maprename(self, req):
        response = MapRenameResponse()
        response.result = self.pgm2png(self.path + req.oldName + '.pgm', self.path)
        if response.result == 'Success':
            self.modify_yaml(self.path + req.oldName + '.yaml')
        return response

    def pgm2png(self, pgm_file, png_dir):
        try:
            pgm = Image.open(pgm_file)
        except IOError:
            rospy.loginfo('file: %s not exit', pgm_file)
            return 'error: file not exit'
        name = (str)(os.path.join(png_dir, os.path.splitext(os.path.basename(pgm_file))[0])) + ".png"
        pgm.save(name)
        os.remove(pgm_file)
        return 'Success'

    def modify_yaml(self, yaml_file):
        lines = open(yaml_file).readlines()
        lines[0] = re.sub("pgm", "png", lines[0])
        open(yaml_file, 'w+').writelines(lines)


if __name__ == "__main__":
    rospy.init_node('map_rename_server')
    pkg_path = rospy.get_param('~api_pkg_path', '/home/whj/catkin_ws/src/api/')
    map_path = pkg_path + 'maps/'
    MapRenameSrv(map_path)
    rospy.spin()
