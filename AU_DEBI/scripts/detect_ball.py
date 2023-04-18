#!/usr/bin/env python3

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image
from  ros_numpy import numpify

class Percept():
    def __init__(self):
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        rospy.loginfo("Finish Initialization of Node ")


    def callback(self, data):
        '''
        get data from image_raw topic
        '''
        rospy.loginfo("Receicing Video Frame")
        self.frame = numpify(data)
        # print(frame)
        # print(frame.shape)
        self.contour()
    
    def hough_circle(self):
        img = self.frame.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # gray = cv2.medianBlur(gray, 5)
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                param1=100, param2=30,
                                minRadius=1, maxRadius=150)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv2.circle(img, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv2.circle(img, center, radius, (255, 0, 255), 3)

        cv2.imshow("Camera", img)
        cv2.waitKey(1)
    
    def contour(self):
        image = self.frame.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find Canny edges
        edged = cv2.Canny(gray, 30, 200)
    
        # Finding Contours
        # Use a copy of the image e.g. edged.copy()
        # since findContours alters the image
        contours, hierarchy = cv2.findContours(edged, 
            cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        

        print("Number of Contours found = " + str(len(contours)))
        
        # Draw all contours
        # -1 signifies drawing all contours
        cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
        
        cv2.imshow('Contours', image)
        cv2.waitKey(1)

    def finish(self):
        cv2.destroyAllWindows()




        
        

# if __name__ == '__main__':
rospy.init_node('stream_sub', anonymous=True)
print("After Node init")
percept = Percept()
rospy.spin()
