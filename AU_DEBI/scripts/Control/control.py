#!/usr/bin/env python
import rospy
import cv2
import time
import numpy as np

from cv_bridge import CvBridge 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler


#LANDMARKS
INIT_POSE     = [0.85, 0, 0]
LINE          = 225    # RED LINE on 2.25
DANGER_AREA   = 180 
DANGER_TIME   = 5
DANGER_TOL    = 20

NUM_BALLS     = 6
TURTLE_RADIUS = 0.3
ROT_SPEED     = 0.5
SEARCH_SPEED  = 0.1
SEARCH_ANGLES = [ -60, -40, -20, 0, 20, 40, 60]
SYS_DELAY     = 0.1
INTERFERECE   = 5   
LIM_ANGLES    = 80

REVIERSE_TIME = 6
ANGLE_DELAY   = 25
# Callback function called whenever
# x-y coordinate received



class Contorl():
    def __init__(self):

        #Balls
        self.home       = []
        self.away       = []
        self.nhome      = 0
        self.naway      = 0
        self.nunseen    = 0
        self.dir_balls  = []
        self.last_search=0


        #PATH PLANNER
        self.planner = MoveBaseActionGoal()
        self.planner.goal.target_pose.header.frame_id    = "map"
        self.Activate_planner = False

        self.pose  = Pose2D()
        self.odom  = Pose2D()
        self.speed = Pose2D()

        # publish to /cmd_vel topic the angular-z velocity change
        self.pub_vel     = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.path_plan   = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=5)
        self.pub_pose2d  = rospy.Publisher('/robot_pose', Pose2D, queue_size=5)
        self.pub_odom2d  = rospy.Publisher('/robot_odom', Pose2D, queue_size=5)
        self.pub_speed2d = rospy.Publisher('/robot_speed', Pose2D, queue_size=5)



        # subscribe to /ball_location topic to receive coordinates
        # rospy.Subscriber("/ball_location",Point, self.drive_callback)
        rospy.Subscriber("/balls",Float32MultiArray, self.get_balls)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.get_pose)
        rospy.Subscriber('/odom', Odometry, self.get_odom)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.get_planner_status)

        
    def continous_search(self):
        rospy.loginfo("Start Searching for Home Balls")

        if self.odom.x > DANGER_AREA:
            # If the Robot Very close to red line reset it's position
            self.set_path(INIT_POSE)


        if self.odom.theta > LIM_ANGLES:
            rospy.loginfo(f"Yaw angle {self.odom.z} Rearange it to 0")
            self.yaw_PControl(0)
            time.sleep(SYS_DELAY)

        

        if self.dir_balls == []:
            if self.odom.theta > 0:
                dir_search = 1
            else:
                dir_search = -1
        else:
            dir_search = -1 * self.dir_balls[-1]

        start = 0
        while self.nhome == 0:
            self.set_speed(yaw= dir_search * SEARCH_SPEED)
            if abs(self.odom.theta*(180/np.pi)) >= LIM_ANGLES and (time.time() - start) > REVIERSE_TIME:
                start       = time.time()
                dir_search *= -1     # Reverse Direction
                rospy.loginfo(f"Reverse Direction of Search")

        self.stop()
        time.sleep(0.1)
        self.dir_balls.append(dir_search)
        rospy.loginfo(f"Finish Searching Found {self.nhome } Home Balls")


    def discrete_search(self):
        rospy.loginfo("Start Discrete Searching for Home Balls")

        if self.odom.x > DANGER_AREA:
            # If the Robot Very close to red line reset it's position
            self.set_path(INIT_POSE)
            # self.receding()


        
        for i in range(self.last_search, len(SEARCH_ANGLES)):
            if self.nhome ==0:
                self.yaw_PControl(SEARCH_ANGLES[i])
                self.last_search = i
                time.sleep(2)
            else:
                break



        self.stop()
        time.sleep(0.1)
        rospy.loginfo(f"Finish Searching Found {self.nhome } Home Balls")




            

    def shoot_rotate(self, dir, speed=10):
        '''
        Shoot ball by rotating around z axis
        '''
        time.sleep(1)
        vel = Twist()
        vel.angular.z = (dir/abs(dir))*speed
        print(vel)
        self.pub_vel.publish(vel)


    def shoot_forward(self, dist, speed=0.3):
        '''
        Shoot ball by rotating around z axis
        '''
        self.stop()
        time.sleep(SYS_DELAY)

        self.set_speed(x=speed)
        rospy.loginfo(f"Start Forward shooting at speed of {speed}")
        start = time.time()
        while (self.odom.x - dist) <  0:
            if time.time() - start > DANGER_TIME:
                break
        
        self.stop()
        rospy.loginfo(f"Finish Forward shooting at speed of {speed}")

            

    def receding(self, x=100):
        self.yaw_PControl(0, tol=1)
        self.x_PControl(x)
        

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
    
    def yaw_PControl(self, angle, tol=5, P=1):
        angle    = angle * (np.pi/180)      #Convert to radian

        rospy.loginfo(f"Setpoint {angle} to P Controller degree")
        while abs(angle - self.odom.theta) > tol*np.pi/180:
            error    = (angle - self.odom.theta)
            P_signal = P * error
            self.set_speed(yaw=P_signal)
            time.sleep(0.05)
        
        rospy.loginfo(f"Rach Setpoint {angle} degree")
        self.stop()

    def yaw_diff_PControl(self, diff_angle, tol=5, P=1):
        angle    =  self.odom.theta * (180/np.pi) + diff_angle
        rospy.loginfo(f"Set point to angle {angle} degree")
        angle    =  angle * (np.pi/180)
        rospy.loginfo(f"Setpoint {angle} degree to P Controller degree")
        while abs(angle - self.odom.theta) > tol*np.pi/180:
            error    = (angle - self.odom.theta)
            P_signal = P * error
            self.set_speed(yaw=P_signal)
            time.sleep(0.05)
        
        rospy.loginfo(f"Rach Setpoint {angle} degree")
        self.stop()
    
    def x_PControl(self, x, P=0.01):
        rospy.loginfo(f"Setpoint {x} to P Controller cm")
        while abs(x - self.odom.x) > 3:
            error    = (x - self.odom.x)
            P_signal = P * error
            self.set_speed(x=P_signal)
            time.sleep(0.05)
        
        rospy.loginfo(f"Rach Setpoint {x} cm")
        self.stop()


   

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


    def check_balls(self):
        '''
        check how many balls seen are in Home or Away
        '''
        self.nhome   = 0
        self.naway   = 0
        self.nunseen = 0

        for ball in self.balls:
            if ball[1] + self.odom.x > LINE:
                self.away.append(ball)
                self.naway+=1
            else:
                self.home.append(ball)
                self.nhome+=1
        self.nunseen = NUM_BALLS - (self.naway + self.nhome)

        rospy.loginfo(f"Seen Balls : {self.naway + self.nhome},  Home : {self.nhome},  Away: {self.naway}")


    def set_speed(self, x=0, y=0, yaw=0):
        time.sleep(SYS_DELAY)
        vel = Twist()
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = yaw
        vel.linear.x  = x
        vel.linear.y  = y
        vel.linear.z  = 0
        self.pub_vel.publish(vel)
    
    def set_path(self, goal=INIT_POSE):
        self.Activate_planner = True
        orientation  = [0, 0, goal[2]]
        planner_orientation = quaternion_from_euler(*orientation)
        self.planner.goal.target_pose.pose.position.x    = goal[0]
        self.planner.goal.target_pose.pose.position.y    = goal[1]
        self.planner.goal.target_pose.pose.orientation.z = planner_orientation[2]
        self.planner.goal.target_pose.pose.orientation.w = planner_orientation[3]   

        self.path_plan.publish(self.planner)
        rospy.loginfo(f"Set PLanner to {goal}")

        time.sleep(SYS_DELAY)
        while self.path_status != 3:
            pass

        rospy.loginfo(f"Path to {goal} has been executed ")

    def get_pose(self, data):
        '''
        get Position in Pose3D and convert it to Pose2D
        1- Convert Qaternion to Euler angle
        2- Take positon x, y amd orientation theta to  pe Pose2D
        3- Publish Pose2D
        '''
        orientation     = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        orientation     = euler_from_quaternion(orientation)
        self.pose.x     = data.pose.pose.position.x * 100
        self.pose.y     = data.pose.pose.position.y * 100
        self.pose.theta = orientation[2]
        self.pub_pose2d.publish(self.pose)

    def get_odom(self, data):
        '''
        Get Odomertry in Pose3D and convert it to  Pose2D and Twist2D
        1- Convert Qaternion to Euler angle
        2- Take positon x, y amd orientation theta to  pe Pose2D
        3- Publish Pose2D
        4- take linear speeds from twist.linear x,y and yaw from twist.angular.z
        5- Publish speed which is also Pose2D
        '''
        orientation     = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        orientation     = euler_from_quaternion(orientation)
        self.odom.x     = data.pose.pose.position.x  * 100
        self.odom.y     = data.pose.pose.position.y  * 100
        self.odom.theta = orientation[2]
        self.speed.x    = data.twist.twist.linear.x
        self.speed.y    = data.twist.twist.linear.y
        self.speed.theta= data.twist.twist.angular.z
        self.pub_odom2d.publish(self.odom)
        self.pub_speed2d.publish(self.speed)

        #SAFETY CONDITION
        if self.odom.x > LINE - DANGER_TOL:
            rospy.loginfo(f"DANGER Robot reach the line ")
            self.stop()
            self.set_path(INIT_POSE)
            

    def get_planner_status(self, data):
        if self.Activate_planner:
            self.path_status = data.status_list[0].status

    
    def get_balls(self, data):
        '''
        get ball and sort it ascendingly by it's y value distance from  robot
        1- Get Row x,y of balls  as [x1,y1,...,xn,yn] 
        2- Convert to list of tuple as [(x1,y1),...,(xn,yn)]
        3- Sort this tuple by y ascendingly
        '''
        self.balls = [(x,y) for x,y in zip(data.data[0::2],data.data[1::2])]
        self.balls.sort(key=lambda a: a[1])
        self.check_balls()

if __name__ == '__main__':

        # intialize the node
        rospy.init_node('robot_control', anonymous=True)
        

        control = Contorl()
        time.sleep(1)

        # control.receding()
        shoooting = 0
        while control.naway != NUM_BALLS:
            if control.nhome != 0:
                control.stop()
                time.sleep(2.5)
                target = control.home[0]
                angle  = -1 * np.arcsin(target[0]/target[1]) * (180/np.pi)  
                target_pose = target[1] + control.odom.x
                rospy.loginfo(f"Target Ball at distance {target[1]} and angle {angle}")
                control.yaw_diff_PControl(angle)
                control.shoot_forward(target_pose)
                shoooting+=1
                control.set_path(INIT_POSE)
                time.sleep(0.5)


            else :
                rospy.loginfo(f"There's no Balls on the scene")
                control.discrete_search()

        
        rospy.loginfo("Robot Pass all Balls in other side")    
        rospy.spin()