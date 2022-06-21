#!/usr/bin/env python

from distutils.file_util import move_file
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time
import tf
import numpy as np
x=0
y=0
yaw=0
quaternion_callback = [0,0,0,0]

def poseCallback(pose_message):
    global x,y,yaw,quaternion_callback
    x=pose_message.pose.pose.position.x
    y=pose_message.pose.pose.position.y
    
    quaternion_callback[0] = pose_message.pose.pose.orientation.x
    quaternion_callback[1] = pose_message.pose.pose.orientation.y
    quaternion_callback[2] = pose_message.pose.pose.orientation.z
    quaternion_callback[3] = pose_message.pose.pose.orientation.w
    rpy=tf.transformations.euler_from_quaternion(quaternion_callback)
    yaw=rpy[2]
    print("posicao atual",x,y)



def go_to_goal(velocity_publisher,xGoal_world,yGoal_world,yawGoal_world):
    global x
    global y, yaw


    robot_yaw = math.radians(-90)
    H_rm = [[math.cos(robot_yaw), -math.sin(robot_yaw), -0.8],
            [math.sin(robot_yaw), math.cos(robot_yaw), 2.35],
            [0, 0, 1]]
    yawGoal_world_rad=math.radians(yawGoal_world)
    H_mg = [[math.cos(yawGoal_world_rad), -math.sin(yawGoal_world_rad), xGoal_world],
            [math.sin(yawGoal_world_rad), math.cos(yawGoal_world_rad), yGoal_world],
            [0, 0, 1]]
    H_rg=np.dot(H_rm,H_mg)
    x_goal = H_rg[0][2]
    y_goal = H_rg[1][2]
    loop_rate = rospy.Rate(1000)
    velocity_message = Twist()
    
    while (True):
        K_linear = 0.5 
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))
        linear_speed = distance * K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        
        loop_rate.sleep()
        print ('x=', x, ', y=',y, ', distance to goal:', distance)


        if (distance <0.01):
            break



if __name__ == '__main__':
    try:
        
        rospy.init_node('tiago_client_py', anonymous=True)

        cmd_vel_topic='/mobile_base_controller/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/robot_pose"
        pose_subscriber = rospy.Subscriber(position_topic, PoseWithCovarianceStamped, poseCallback) 
        time.sleep(2)
        
        go_to_goal(velocity_publisher,1.0, 1.4,270)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")