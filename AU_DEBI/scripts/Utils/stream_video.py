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
        size = (640, 480)
   
        # Below VideoWriter object will create
        # a frame of above defined The output 
        # is stored in 'filename.avi' file.
        self.result = cv2.VideoWriter('filename.avi', 
                         cv2.VideoWriter_fourcc(*'MJPG'),
                         10, size)

        rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        rospy.loginfo("Finish Initialization of Node ")


    def callback(self, data):
        '''
        get data from image_raw topic
        '''
        rospy.loginfo("Receicing Video Frame")
        self.frame = numpify(data)
        # print(self.frame.shape)
        
        self.result.write(self.frame)

        cv2.imshow("frame", self.frame)
        cv2.waitKey(1) 
        if self.counter==500:
            rospy.loginfo("Receicing Video Frame")
            self.result.release()
        self.counter+=1
        
   

    def finish(self):
        cv2.destroyAllWindows()




        
        

# if __name__ == '__main__':
rospy.init_node('save_frames', anonymous=True)
print("After Node init")
percept = SaveFrames()
rospy.spin()
