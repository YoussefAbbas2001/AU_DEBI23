#!/usr/bin/env python
import numpy as np
import cv2
import rospy
import time
from move_base_msgs.msg import MoveBaseActionGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_msgs.msg import TFMessage





def get_tf(data):
    print(data.transforms)

rospy.init_node("set_planner")
rospy.Subscriber("/tf", TFMessage, get_tf)

rospy.spin()