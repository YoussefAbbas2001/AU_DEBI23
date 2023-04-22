#!/usr/bin/env python3
import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from  ros_numpy import numpify

class SaveFrames():
    def __init__(self):
        self.steps=60
        self.counter=0
        self.images=0
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        rospy.loginfo("Finish Initialization of Node ")


    def callback(self, data):
        '''
        get data from image_raw topic
        '''
        rospy.loginfo("Receicing Video Frame")
        self.frame = numpify(data)
        # print(self.frame.shape)
        
        if self.counter % self.steps ==0:
            cv2.imwrite(f"/home/abbas/DEBI/src/AU_DEBI/imgs/{self.images:03}.jpg",self.frame)
            rospy.loginfo(f"Save frame {self.images}")
            self.images+=1

        cv2.imshow("frame", self.frame)
        cv2.waitKey(1)    
        self.counter+=1
   

    def finish(self):
        cv2.destroyAllWindows()




        
        

# if __name__ == '__main__':
rospy.init_node('save_frames', anonymous=True)
print("After Node init")
percept = SaveFrames()
rospy.spin()
