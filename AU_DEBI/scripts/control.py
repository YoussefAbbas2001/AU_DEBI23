#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge 

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import time


TURTLE_RADIUS = 0.3
ROT_SPEED     = 0.5
SYS_DELAY     = 0.1
# Callback function called whenever
# x-y coordinate received


class Contorl():
    def __init__(self):
        # subscribe to /ball_location topic to receive coordinates
        rospy.Subscriber("/ball_location",Point, self.drive_callback)

        # publish to /cmd_vel topic the angular-z velocity change
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)


    def shoot(self, dir, speed=10):
        '''
        Shoot ball by rotating around z axis
        '''
        time.sleep(0.5)
        vel = Twist()
        vel.angular.z = (dir/abs(dir))*speed
        print(vel)
        self.pub_vel.publish(vel)

    def stop(self):
        '''
        STOP turtlebot 
        '''
        vel = Twist()
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0
        vel.linear.x  = 0
        vel.linear.y  = 0
        vel.linear.z  = 0
        self.pub_vel.publish(vel)

    def drive_callback(self, data):
        global vel
        ball_x 	= data.x
        ball_y 	= data.y
        
        
        # Create Twist() instance
        vel = Twist()

        # 
        if ball_x < 0 and ball_y < 0:
            vel.angular.z = 0
        else:
            # Determine center-x, normalized deviation from center
            mid_x  	= int(TURTLE_RADIUS/2)
            delta_x	= ball_x - mid_x
            norm_x 	= delta_x/TURTLE_RADIUS

            if norm_x > TURTLE_RADIUS:
                print ("delX: {:.3f}. Turn right".format(norm_x))
                vel.angular.z = -ROT_SPEED
            elif norm_x < -TURTLE_RADIUS:
                print ("delX: {:.3f}. Turn left".format(norm_x))
                vel.angular.z = ROT_SPEED
            if abs(norm_x) < TURTLE_RADIUS:
                print ("delX: {:.3f}. Stay in center".format(norm_x))
                vel.angular.z = 0
        # publish vel on the publisher
        self.pub_vel.publish(vel)

if __name__ == '__main__':

        # intialize the node
        rospy.init_node('drive_wheel', anonymous=True)
        

        control = Contorl()
        control.shoot(dir=-1)
        time.sleep(1)
        control.stop()
        

        rospy.spin()