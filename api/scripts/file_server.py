#! /usr/bin/env python

import rospy
import base64
import os
import json
from api_msgs.srv import GetList, GetListResponse,  DelFile, DelFileResponse, DownloadFile, DownloadFileResponse
from api_msgs.srv import DownloadMap, DownloadMapResponse, UploadMap, UploadMapResponse

class FileSrv():
    def __init__(self, pkg_path, ip):
        self.pkg_path = pkg_path
        self.server_ip = ip
        rospy.Service('api/get_bag_list', GetList, self.handle_getbaglist)
        rospy.Service('api/del_bag', DelFile, self.handle_delbag)
        rospy.Service('api/download_bag', DownloadFile, self.handle_downloadbag)
        rospy.Service('api/download_map', DownloadMap, self.handle_downloadmap)
        rospy.Service('api/upload_map', UploadMap, self.handle_uploadmap)

    def handle_getbaglist(self, req):
        bag_path = os.path.join(self.pkg_path, 'bags/')
        f_list = os.listdir(bag_path)
        bag_dict = {}
        for f in f_list:
            if os.path.splitext(f)[1] == '.bag':
                bag_dict[f] = os.path.getsize(bag_path+f)

        res = GetListResponse()
        res.list = json.dumps(bag_dict)
        res.msg = 'success'
        return res

    def handle_delbag(self, req):
        res = DelFileResponse()
        rospy.loginfo('request to delte bag "%s"', req.name)
        bag_file = os.path.join(self.pkg_path, 'bags/' + req.name)
        try:
            os.remove(bag_file)
            res.msg = 'success'
        except Exception as e:
            res.msg = str(e)
        return res

    def handle_downloadbag(self, req):
        res = DownloadFileResponse()
        bag_file = os.path.join(self.pkg_path, 'bags/' + req.name)
        # try:
        #     res.size = os.path.getsize(bag_file)
        #     with open(bag_file, 'rb') as f:
        #         res.base64Data = str(base64.b64encode(f.read()))
        #     res.msg = 'success'
        # except Exception as e:
        #     res.msg = str(e)

        try:
            res.size = os.path.getsize(bag_file)
            #res.data = self.convertFileToOneAndZero(bag_file)
            res.link= self.server_ip + ':80/bags/'+req.name
            res.msg = 'success'
        except Exception as e:
            res.msg = str(e)

        return res

    def handle_downloadmap(self, req):
        res = DownloadMapResponse()
        # png_file = os.path.join(self.pkg_path, 'maps/' + req.name + '.png')
        # yaml_file = os.path.join(self.pkg_path, 'maps/' + req.name + '.yaml')
        # try:
        #     with open(png_file, 'rb') as f:
        #         res.mapFile = str(base64.b64encode(f.read()))
        #         #res.mapFile = f.read()
        #     with open(yaml_file, 'rb') as f:
        #         res.yamlFile = str(base64.b64encode(f.read()))
        #     res.msg = 'success'
        # except Exception as e:
        #     res.msg = str(e)
        # res.mapFile = self.convertFileToOneAndZero(png_file)
        # res.yamlFile = self.convertFileToOneAndZero(yaml_file)
        map_path = os.path.join(self.pkg_path, 'maps/'+req.name+'.yaml')
        if not os.path.exists(map_path):
            res.msg = 'map not exist'
            return res

        res.mapFile = self.server_ip + ':80/maps/' + req.name + '.png'
        res.yamlFile = self.server_ip + ':80/maps/' + req.name + '.yaml'
        if req.is_download_goals.upper() == 'TRUE':
            res.goalsFile = self.server_ip + ':80/goals/' + req.name + '.json'

        res.msg = 'success'

        return res

    def handle_uploadmap(self, req):
        res = UploadMapResponse()
        map_file = os.path.join(self.pkg_path, 'maps/' + req.name + '.png')
        yaml_file = os.path.join(self.pkg_path, 'maps/' + req.name + '.yaml')

        res.msg = self.convertOneAndZeroToFile(req.map_data, map_file)
        if res.msg == 'success':
            res.msg = self.convertOneAndZeroToFile(req.yaml_data, yaml_file)
            res.mapSize = os.path.getsize(map_file)
            res.yamlSize = os.path.getsize(yaml_file)

        return res

    # file to 1 0 to string
    def convertFileToOneAndZero(self, path):
        try:
            with open(path, 'rb') as f:
                src = f.read()
        except Exception as e:
            return str(e)

        result = []
        for i in src:
            temp = bin(ord(i))[2:]
            temp = '0' * (8-len(temp)) + temp
            result.append(temp)

        return ''.join(result)

    # src: 101001 string
    # path: path save the output file
    def convertOneAndZeroToFile(self, src, path):
        result = []
        if os.path.exists(path):
            return 'file exist'

        for i in range(0, len(src), 8):
            result.append(chr(int(src[i:i+8], 2)))

        with open(path, 'wb') as f:
            f.write(''.join(result))
        return 'success'


if __name__ == "__main__":
    rospy.init_node('file_server')
    pkg_path = rospy.get_param('~api_pkg_path', '/home/whj/catkin_ws/src/api/api/')
    ip = rospy.get_param('~server_ip', '192.168.10.155')
    FileSrv(pkg_path, ip)
    rospy.spin()
