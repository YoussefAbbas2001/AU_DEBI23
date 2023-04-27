#!/usr/bin/env python3

import numpy as np
import cv2
import rospy
import time
import tf2_ros
import tf2_geometry_msgs

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, Point
from std_msgs.msg import Float32MultiArray
from  ros_numpy import numpify
from ultralytics import YOLO

class Percept():
    def __init__(self):
        self.camera_height = 12   #cm
        self.alpha         = 65.5 #degree
        self.pix_y         = 480  #vertical fov pixel
        self.pix_x         = 640  #horizontalfov pixel
        self.vfov          = 48.8 #vertical fov in degree
        self.hfov          = 62.2 #horizontal fov in degree

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.model = YOLO("/home/abbas/DEBI/src/AU_DEBI/weights/gazebo12_416.pt")
        self.colors = [(50, 50, 255), (255, 50, 50), (50, 255, 50), (180, 100, 180)]

        self.balls =rospy.Publisher('/balls', Float32MultiArray,queue_size=10)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.get_pose)
        rospy.loginfo("Finish Initialization of Node ")


    def callback(self, data):
        '''
        get data from image_raw topic
        '''
        rospy.loginfo("Receicing Video Frame")
        self.frame = numpify(data)
        self.gray  = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        self.yolo()

    def get_pose(self, data):
        self.robot_x     = data.pose.pose.position.x
        self.robot_y     = data.pose.pose.position.y
        self.robot_theta = data.pose.pose.orientation.z

        rospy.loginfo(f"x : {self.robot_x}, y : {self.robot_y}, theta : {self.robot_theta}")

    def reference_pose(self, x_img, y_img):
        '''
        ABOUT:
            get the polar coordinate of point in surface plane with respect to robot

        INPUT:
            x,y  : pixel positions of point in frame

        OUTPUT:
            dist : distance of point from robot in cm
            angle: angle from robot elevation axis in degree  
        '''
        base_to_map = self.tfBuffer.lookup_transform("map", "base_footprint", rospy.Time.now(), rospy.Duration(1.0))
        
        beta = (np.abs(y_img-self.pix_y)/self.pix_y) * self.vfov 
        theta= self.alpha + beta 
        y = self.camera_height * (np.tan(theta*(np.pi/180))) 
        angle= ((x_img-(self.pix_x/2))/(self.pix_x/2)) * self.hfov
        x    = y * np.sin(angle*(np.pi/180))

        base_header = base_to_map.header
        base_header.frame_id = 'base_footprint'
        base_point = PointStamped(header=base_header, point=Point(x,y,0))
        map_point = tf2_geometry_msgs.do_transform_point(base_point, base_to_map)
        # print(base_to_map.header)
        # print(map_point)
        # print(base_point)
        # print(dir(tf2_geometry_msgs))


        return (x, y)

    def yolo(self, iou=0.5, conf=0.40, duration=1000,classes=[0], save=False, log_path="Perception/logs"):
        '''
        ABOUT: 
            This function is for display frame with detection on it
            for debugging
        '''
        frame = self.frame
        # frame = self.gray
        results = self.model.predict(frame, iou=iou, conf=conf, imgsz=416,classes=classes,hide_labels = True, verbose=False, show=True)   #infer frame objects
        result = list(results)[0]
        boxes  =  result.boxes

        balls    = []
        # balls_pub= Float32MultiArray() 
        for box in boxes:
            xywh  = box.xywh[0].numpy().astype('int16')
            cf_x, cf_y  = (xywh[0]  , xywh[1]+xywh[3]/2)
            (x, y)= self.reference_pose(cf_x, cf_y)
            cls  = box.cls.numpy()
            
            if cls == 0:
                balls.extend([x,y])
                print(f'x={x}, y={y}' )
        
        self.balls.publish(Float32MultiArray(data=balls))
        if save:
            cv2.imwrite(f"{log_path}/img_{time.time()}.jpg", frame)


    
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
