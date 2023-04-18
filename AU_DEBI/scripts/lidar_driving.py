#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan


import ros_numpy
import cv2
import os
import numpy as np
import time

num_points = 360
angle_resolution = (1/num_points) * 360
PPD        = 1/angle_resolution

class LidarDrive():
    def __init__(self) :
        rospy.Subscriber('scan', LaserScan, self.get_laser )
        self.ranges = [0]*num_points

    def get_laser(self, val):
        self.ranges = np.array(val.ranges)

    def get_distance(self, angle):
        return self.ranges[angle]




rospy.init_node("Laser_Scan_Custom")
lidar = LidarDrive()
time.sleep(1)
print(lidar.get_distance(0))
print(lidar.get_distance(180))
rospy.spin()