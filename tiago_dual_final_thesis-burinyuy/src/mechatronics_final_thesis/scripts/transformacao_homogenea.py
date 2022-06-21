#!/usr/bin/env python  
from cmath import cos
import rospy
import numpy as np
import actionlib
import tf
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

def move_to_goal(xGoal_world,yGoal_world,yawGoal_degree):
    robot_yaw = math.radians(-90)
    H_rm = [[math.cos(robot_yaw), -math.sin(robot_yaw), -0.8],
            [math.sin(robot_yaw), math.cos(robot_yaw), 2.35],
            [0, 0, 1]]
    yawGoal=math.radians(yawGoal_degree)
    H_mg = [[math.cos(yawGoal), -math.sin(yawGoal), xGoal_world],
            [math.sin(yawGoal), math.cos(yawGoal), yGoal_world],
            [0, 0, 1]]
    H_rg=np.dot(H_rm,H_mg)
    print("H = ",H_rg)
    print("x = ",H_rg[0][2])
    print("y = ",H_rg[1][2])
    print("yaw = ", math.asin(H_rg[1][0]))



if __name__ == '__main__':
   
   xGoal_world = 1
   yGoal_world = 0.8
   yawGoal_degree = 0
   move_to_goal(xGoal_world,yGoal_world,yawGoal_degree)
   rospy.spin()
