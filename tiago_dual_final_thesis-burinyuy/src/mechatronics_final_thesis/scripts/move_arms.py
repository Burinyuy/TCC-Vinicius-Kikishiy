#!/usr/bin/env python

import rospy
from moveit_python import *
import tf
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
rospy.init_node('tiago_client_py')
    
def move_arm(x,y,z,roll,pitch,yaw,wait):
    arm_goal = PoseStamped() 
    quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

    arm_goal.header.frame_id="base_footprint" 
    arm_goal.header.stamp = rospy.Time.now() 
    arm_goal.pose.position.x = x 
    arm_goal.pose.position.y = y
    arm_goal.pose.position.z = z
    arm_goal.pose.orientation.x=quaternion[0] 
    arm_goal.pose.orientation.y=quaternion[1]
    arm_goal.pose.orientation.z=quaternion[2]
    arm_goal.pose.orientation.w = quaternion[3]
    group= MoveGroupInterface("arm_"+side, "base_footprint")
    group.moveToPose(arm_goal,"gripper_"+side+"_grasping_frame",wait=wait) 
    print("Move "+side+" arm")

    if(y>0):
        side = "left" 
    else:
        side = "right"

    group= MoveGroupInterface("arm_"+side, "base_footprint") 
    group.moveToPose(arm_goal,"gripper_"+side+"_grasping_frame",wait=wait)
    print("Move "+side+" arm") 

if __name__ == '__main__':
   rospy.init_node('map_navigation', anonymous=False)
 
    move_arm(0.005, 0.613, 1.263, 0.872665, -1.48353, 0.610865,True)
    move_arm(0.005, -0.613, 1.263, 0.872665, -1.48353, 0.610865,True)
    rospy.spin()
