#!/usr/bin/env python3
import sys
import rospy
import cv2
import time 
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from  ros_numpy import numpify

from ultralytics import YOLO


IMAGE_SHAPE   = (640,300)
BLUE_LOWER    = (100, 100, 100)
BLUE_UPPER    = (150, 255, 255)
RED_LOWER1     = (170, 100, 100)
RED_UPPER1     = (180, 255, 255)
RED_LOWER2     = (0, 100, 100)
RED_UPPER2     = (15, 255, 255)
AREA_LOWER    = 30
AREA_UPPER    = 10000

MIN_DISTANCE  = 240       # min distance to ball in pixels
MAX_SIZE      = 300       # max size in front of ball
COLOR         = (0, 0, 255)

# SPEEDS
SEARCHING_SPEED = 0.1
AIMING_SPEED    = 0.15

# LIMITIS
STRIGHT_LIMIT   = 100

class TakePhoto:
    def __init__(self):

        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.rate=rospy.Rate(1)
        self.rot=Twist()
        
        self.ball_is_taken=False

        self.cx=0
        self.cy=0

        self.bridge = CvBridge()
        self.image_received = False

        self.camera_height = 12   #cm
        self.alpha         = 65.5 #degree
        self.pix_y         = 480  #vertical fov pixel
        self.pix_x         = 640  #horizontalfov pixel
        self.vfov          = 48.8 #vertical fov in degree
        self.hfov          = 62.2 #horizontal fov in degree


        self.model = YOLO("src/AU_DEBI/weights/gazebo1.pt")
        self.colors = [(50, 50, 255), (255, 50, 50), (50, 255, 50), (180, 100, 180)]

        rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        rospy.loginfo("Finish Initialization of Node ")

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):
        '''
        get data from image_raw topic
        '''
        rospy.loginfo("Receicing Video Frame")
        self.frame = numpify(data)
        self.yolo()

        
        if (not self.ball_is_taken):
            self.yolo()
            # self.find_ball(self.frame)
        else:
            self.yolo()
            # self.find_goal(self.frame)

        # self.move_to_object()


    def reference_pose(self, x, y):
        '''
        ABOUT:
            get the polar coordinate of point in surface plane with respect to robot

        INPUT:
            x,y  : pixel positions of point in frame

        OUTPUT:
            dist : distance of point from robot in cm
            angle: angle from robot elevation axis in degree  
        '''
        beta = (np.abs(y-self.pix_y)/self.pix_y) * self.vfov 
        theta= self.alpha + beta 
        dist = self.camera_height * (np.tan(theta*(np.pi/180))) 
        angle= ((x-(self.pix_x/2))/(self.pix_x/2)) * self.hfov

        return (dist, angle)

    def yolo(self, iou=0.5, conf=0.5, duration=1000,classes=[0, 1, 2], save=False, log_path="Perception/logs"):
        '''
        ABOUT: 
            This function is for display frame with detection on it
            for debugging
        '''
        frame = self.frame
        results = self.model.predict(frame, iou=iou, conf=conf, classes=classes, show=True)   #infer frame objects
        result = list(results)[0]
        boxes  =  result.boxes

        CX, CY, W, H = 0, 0, 0, 0
        for box in boxes:
            xywh = box.xywh[0].numpy().astype('float32')
            cls  = box.cls.numpy()
            if xywh[2] * xywh[3] > W * H:
                CX, CY, W, H = xywh
        
        
        self.move_to_object(CX)

    

    def check_passed(self, x, y):
        for i in range(y):
            if  self.pix_inrange(self.hsv_frame[y, x][0], RED_LOWER1[0], RED_UPPER1[0]) or self.pix_inrange(self.hsv_frame[y, x][0], RED_LOWER2[0], RED_UPPER2[0]) \
                and self.pix_inrange(self.hsv_frame[y, x][1], RED_LOWER1[1], RED_UPPER1[1]) or self.pix_inrange(self.hsv_frame[y, x][1], RED_LOWER2[1], RED_UPPER2[1]) \
                and self.pix_inrange(self.hsv_frame[y, x][2], RED_LOWER1[2], RED_UPPER1[2]) or self.pix_inrange(self.hsv_frame[y, x][2], RED_LOWER2[2], RED_UPPER2[2]):
                  return True
        
        return False



    def show_image(self,img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)
        


        
    def find_ball(self,cx, cy, W):
      
        
        if(W>MAX_SIZE and self.cy>MIN_DISTANCE):
            self.ball_is_taken=True
            print("Taken")

        print(f'x = {self.cx}, y = {self.cy}, W = {W}')


    def move_to_object(self, cx ):
        '''
        Move the turtlebot depend on the ball position

        '''
        if(cx==0):
            print("searching")
            self.rot.angular.z= SEARCHING_SPEED
            self.rot.linear.x = 0

        else:
        
            obj_x = self.cx - IMAGE_SHAPE[0]//2

            if(obj_x<=STRIGHT_LIMIT and obj_x>=-STRIGHT_LIMIT):
                print("straight")
                self.rot.angular.z = 0
                self.rot.linear.x  = AIMING_SPEED
            elif(obj_x>STRIGHT_LIMIT):
                print("Left")
                self.rot.angular.z = -AIMING_SPEED
                self.rot.linear.x  = 0
            elif(obj_x<-STRIGHT_LIMIT):
                print("Right")
                self.rot.angular.z = AIMING_SPEED
                self.rot.linear.x  = 0


        self.pub.publish(self.rot)
     

    def stop(self):
        '''
        Stop ROBOT by setting speeds to zeros
        '''
        self.rot.angular.z=0
        self.rot.linear.x=0
        self.pub.publish(self.rot)
        print("Stopping")
    
    def pix_inrange(self, pix, lower, upper):
        if pix > lower and pix < upper: 
            return True
        else:
            return False


if __name__ == '__main__':

    # Initialize
    rospy.init_node('take_photo', anonymous=False)
    camera = TakePhoto()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        rospy.spin()

    camera.stop()
