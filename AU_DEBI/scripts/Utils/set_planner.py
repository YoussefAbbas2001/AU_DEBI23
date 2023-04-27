#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import time
from move_base_msgs.msg import MoveBaseActionGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node("set_planner")
pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)

x_init  = 0.75
y_init  = 0
yaw_init= 0
orientation = [0, 0, yaw_init]

planner_qurt = quaternion_from_euler(*orientation)
print(planner_qurt)

planner = MoveBaseActionGoal()
planner.goal.target_pose.header.frame_id    = "map"
planner.goal.target_pose.pose.position.x    = x_init
planner.goal.target_pose.pose.orientation.z = planner_qurt[2]
planner.goal.target_pose.pose.orientation.w = planner_qurt[3]

print(planner)
time.sleep(0.5)
pub.publish(planner)

rospy.spin()