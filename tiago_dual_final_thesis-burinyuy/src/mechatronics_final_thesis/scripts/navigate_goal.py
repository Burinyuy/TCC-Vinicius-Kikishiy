#!/usr/bin/env python  
import rospy
import actionlib
import tf
import math
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point



def move_to_goal(xGoal_world,yGoal_world,yawGoal_world):

        robot_yaw = math.radians(-90)
        H_rm = [[math.cos(robot_yaw), -math.sin(robot_yaw), -0.8],
                [math.sin(robot_yaw), math.cos(robot_yaw), 2.35],
                [0, 0, 1]]
        yawGoal_world_rad=math.radians(yawGoal_world)
        H_mg = [[math.cos(yawGoal_world_rad), -math.sin(yawGoal_world_rad), xGoal_world],
                [math.sin(yawGoal_world_rad), math.cos(yawGoal_world_rad), yGoal_world],
                [0, 0, 1]]
        H_rg=np.dot(H_rm,H_mg)
        xGoal = H_rg[0][2]
        yGoal = H_rg[1][2]
        if(yawGoal_world>=0) and (yawGoal_world<=180):
                yawGoal = math.asin(H_rg[1][0])
        else:
                yawGoal = math.acos(H_rg[0][0])


        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
                rospy.loginfo("Waiting for the move_base action server to come up")

        goal = MoveBaseGoal()
   
   

        goal.target_pose.header.frame_id = "map" 
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)

        quaternion = tf.transformations.quaternion_from_euler(0, 0, yawGoal)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        teste = tf.transformations.euler_from_quaternion(quaternion)
        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
                rospy.loginfo("You have reached the destination ")
                print(quaternion)
                print(xGoal)
                print(yGoal)
                return True

        else:
                rospy.loginfo("The robot failed to reach the destination")
                return False

if __name__ == '__main__':
        try:
                rospy.init_node('map_navigation', anonymous=False)

                dinner_table = [2.7,1.5,0]
                purchase_table = [0.9,1.4,-90]
                print('start go to goal')
                move_to_goal(1.25,4,90)
                rospy.spin()
        except rospy.ROSInterruptException:
                rospy.loginfo("node terminated.")
