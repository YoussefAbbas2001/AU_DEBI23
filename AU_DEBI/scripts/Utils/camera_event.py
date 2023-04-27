#!/usr/bin/env python3

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from  ros_numpy import numpify


# Creating mouse callback function  
def draw_circle(event,x,y,flags,param):  
    if(event == cv2.EVENT_LBUTTONDBLCLK):  
        # cv2.circle(self.frame,(x,y),100,(255,255, 0),-1)  
        print(f'x={x}, y={y}')
        
class Event():
    def __init__(self):
        cv2.setMouseCallback('image',draw_circle)  

        rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        rospy.loginfo("Finish Initialization of Node ")


    def callback(self, data):
        '''
        get data from image_raw topic
        '''
        rospy.loginfo("Receicing Video Frame")
        self.frame = numpify(data)
        cv2.imshow("image", self.frame)
        cv2.waitKey(1)




# cv2.namedWindow('image')  
rospy.init_node('camera_event', anonymous=True)
rospy.loginfo("After Node init")
percept = Event()
rospy.spin()