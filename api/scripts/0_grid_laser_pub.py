#!/usr/bin/env python
import rospy
import math
import tf
import numpy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from api.msg import GridLaser, GridPoint
map_meta = OccupancyGrid()
laser_data = LaserScan()
#offset from laser to base_link
X_OFFSET = 0.21
Y_OFFSET = 0.0
#map callback, update map data,actually, map data need no update
def map_callback(msg):
    global map_meta
    map_meta.info = msg.info

#laser callback, update laser data
def laser_callback(msg):
    global laser_data
    laser_data.angle_increment =msg.angle_increment
    laser_data.angle_max = msg.angle_max
    laser_data.angle_min = msg.angle_min
    laser_data.ranges = msg.ranges

def grid_laser_pub():
    rospy.init_node('gird_laser_pub')

    listener=tf.TransformListener()
    rospy.Subscriber('map',OccupancyGrid,map_callback)
    rospy.Subscriber('scan',LaserScan, laser_callback)
    laser_pub=rospy.Publisher('grid_laser',GridLaser,queue_size=1)

    grid_laser = GridLaser()
    global map_meta
    global laser_data
    rate=rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            (trans,rot)=listener.lookupTransform('/map','/base_link',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #calc laser point in base_link frame
        laser_array=numpy.zeros((len(laser_data.ranges),4))
        index=0
        for r in laser_data.ranges:
            laser_array[index][0]=r*math.cos(laser_data.angle_min+laser_data.angle_increment*index)+X_OFFSET
            laser_array[index][1]=r*math.sin(laser_data.angle_min+laser_data.angle_increment*index)+Y_OFFSET
            laser_array[index][3]=1
            index=index+1
        #calc rotation matrix
        trans_mat=numpy.zeros((3,4))
        trans_mat[0][0]=rot[3]*rot[3]-rot[2]*rot[2]
        trans_mat[0][1]=-2*rot[3]*rot[2]
        trans_mat[0][3]=trans[0]
        trans_mat[1][0]=2*rot[3]*rot[2]
        trans_mat[1][1]=rot[3]*rot[3]-rot[2]*rot[2]
        trans_mat[1][3]=trans[1]
        trans_mat[2][2]=1

        #calc laser points in world frame
        if map_meta.resolution != 0.000:
            for point in laser_array:
                grid_point = GridPoint()
                x = trans_mat[0][0]*point[0] + trans_mat[0][1]*point[1] + trans_mat[0][3]*point[3]
                y = trans_mat[1][0]*point[0] + trans_mat[1][1]*point[1] + trans_mat[1][3]*point[3]
                grid_point.x = (int)((x-map_meta.origin.position.x)/map_meta.resolution)
                grid_point.y = (int)((y-map_meta.origin.position.y)/map_meta.resolution)
                grid_laser.gridPnt.append(grid_point)

            grid_laser.mapGridWidth = map_meta.width
            grid_laser.mapGridHeight = map_meta.height

        #publish grid_laser
        laser_pub.publish(grid_laser)
        grid_laser.gridPnt=[]
        rate.sleep()
    rospy.spin()

if __name__=='__main__':
    grid_laser_pub()
