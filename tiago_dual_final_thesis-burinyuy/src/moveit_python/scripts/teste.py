#!/usr/bin/env python
import sys
import os
import glob
import turtle
import sys
import copy
import rospy
from move_python import *
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal,JointTolerance
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import moveit_commander 
import moveit_msgs.msg 
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalID
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import String, Header
from moveit_commander.conversions import pose_to_list
rospy.init_node('tiago_client_py')
    
def move_arm(x,y,z,wait):
        
    arm_goal = PoseStamped() # Creamos un mensaje de tipo PoseStamped
    quaternion = tf.transformations.quaternion_from_euler(0,0,0)

    arm_goal.header.frame_id="base_footprint" #we use the map frame because we specify a global position in the robot
    arm_goal.header.stamp = rospy.Time.now() #set the timestamp of the message equal to the current time using rows by the time
    arm_goal.pose.position.x = x # Definimos la posicion deseada para arm_tool
    arm_goal.pose.position.y = y
    arm_goal.pose.position.z = z
    arm_goal.pose.orientation.x=quaternion[0] # Definimos la orientacion deseada para arm_tool
    arm_goal.pose.orientation.y=quaternion[1]
    arm_goal.pose.orientation.z=quaternion[2]
    arm_goal.pose.orientation.w = quaternion[3]
    group= MoveGroupInterface("arm_torso", "base_footprint") # Creamos una interfaz de MoveIt definiendo - el grupo a planear y la referencia
    group.moveToPose(arm_goal,"arm_tool_link",wait=wait) # Mandamos el goal con el link que alcanzara  - dicha posicion
    print("Moved arm or unreachable") # Confirmamos por consola

if __name__ == '__main__':
   #rospy.init_node('map_navigation', anonymous=False)
   #specify the cordinates of the goal location
 
   move_arm(0,-0.5,0.9,True)
   rospy.spin()
