#!/usr/bin/env python3
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


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

        # Connect image topic
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self, data):
        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image
        
        if (not self.ball_is_taken):
            self.find_ball(cv_image)
        else:
            self.find_goal(cv_image)

        self.move_to_object()

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
        
    def find_goal(self,img):
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hsv_frame = cv2.resize(hsv_frame,IMAGE_SHAPE)
        img = cv2.resize(img,IMAGE_SHAPE)

        mask_frame=cv2.inRange(hsv_frame, BLUE_LOWER, BLUE_UPPER)
        cv2.imshow("mask",mask_frame)
        contours, hierarchy = cv2.findContours(mask_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        X,Y,W,H=0,0,0,0


        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            
            if(area > 30):
                
                x, y, w, h = cv2.boundingRect(contour)
                if(w*h>W*H):
                    X, Y, W, H= x, y, w, h

        img = cv2.rectangle(img, (X, Y),(X + W, Y + H), COLOR, 2)

        self.cx = X+(W/2)
        self.cy = Y+(W/2)
        
        print("to goal")
        print(self.cx)
        cv2.imshow("window", img)
        cv2.waitKey(3)
        
        
    def find_ball(self,img):
        '''
        Find Ball by color
        '''
        self.hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        self.hsv_frame = cv2.resize(self.hsv_frame,IMAGE_SHAPE)
        img = cv2.resize(img, IMAGE_SHAPE)



        mask_frame=cv2.inRange(self.hsv_frame,  BLUE_LOWER, BLUE_UPPER)
        cv2.imshow("mask",mask_frame)
        contours, hierarchy = cv2.findContours(mask_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #_, contours, _= cv2.findContours(mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        X,Y,W,H=0,0,0,0


        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            
            if(area > AREA_LOWER and area < AREA_UPPER)  :
                # print(f'area={area}')
                '''
                If Area of contour in Range then we will assume it'll be ball
                '''
                x, y, w, h = cv2.boundingRect(contour)
                if(w*h>W*H) and self.check_passed(x,y):
                    '''
                    get the largest contour area
                    '''
                    X, Y, W, H= x, y, w, h

        img = cv2.rectangle(img, (X, Y),(X + W, Y + H),(0, 0, 255), 2)

        self.cx = X+(W/2)
        self.cy = Y+(W/2)
        
        if(W>MAX_SIZE and self.cy>MIN_DISTANCE):
            self.ball_is_taken=True
            print("Taken")

        print(f'x = {self.cx}, y = {self.cy}, W = {W}')
        cv2.imshow("window", img)
        cv2.waitKey(3)

    def move_to_object(self):
        '''
        Move the turtlebot depend on the ball position

        '''
        if(self.cx==0):
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
