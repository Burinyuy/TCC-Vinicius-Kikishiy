#!/usr/bin/env python

import rospy
from moveit_python import *
import tf
import time
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker
rospy.init_node('tiago_client_py')

x_c=0
y_c=0
z_c=0
quaternion_callback=[0,0,0,0]
def poseCallback(pose_message):
    global x_c,y_c,z_c,quaternion_callback
    x_c=pose_message.pose.position.x
    y_c=pose_message.pose.position.y
    z_c=pose_message.pose.position.z
    quaternion_callback[0] = pose_message.pose.orientation.x
    quaternion_callback[1] = pose_message.pose.orientation.y
    quaternion_callback[2] = pose_message.pose.orientation.z
    quaternion_callback[3] = pose_message.pose.orientation.w
    print("posicao do cilindro x: ", x_c,", y: ",y_c,", z:",z_c)

def go_arm(start):
    global xc,yc,zc
    time.sleep(4)


    if start=="go" and z_c!=0:
        xc=x_c-0.2
        yc=y_c
        zc=z_c  
        print("posicao recebida x: ", xc,", y: ",yc,", z:",zc) 
    
        if(yc>0):
            move_arm(0.005, 0.613, 1.263, 0.863, -1.48, 0.614,True)
            move_arm(0.378, 0.138, 1.1, 1.5707, 0, 0,True)
        else:
            move_arm(0.005, -0.613, 1.263, 0.863, -1.48, 0.614,True)
            move_arm(0.378, -0.138, 1.1, 1.5707, 0, 0,True)
        move_arm(0.5, yc, zc, 1.5707, 0, 0,True)
        move_arm(xc, yc, zc, 1.5707, 0, 0,True)

def back_arm(start):
    xc=0.5339473143683795
    yc=0.09336445694189038
    if start=="go":
      
        print("posicao recebida back x: ", xc,", y: ",yc ) 
        move_arm(xc, yc, 1.15, 1.5707, 0, 0,True)
        if(yc>0):
            move_arm(0.378, 0.138, 1.1, 1.5707, 0, 0,True)
            
        else:
            move_arm(0.378, -0.138, 1.15, 1.5707, 0, 0,True)

            
def move_arm(x,y,z,roll,pitch,yaw,wait):
    print("start moving")   
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
    if(y>0):
        side = "left" 
    else:
        side = "right"

    group= MoveGroupInterface("arm_"+side, "base_footprint")

    group.moveToPose(arm_goal,"gripper_"+side+"_grasping_frame",wait=wait)
    print("Move "+side+" arm") 

if __name__ == '__main__':

    position_topic = "/cylinder_detector/marker"
    pose_subscriber = rospy.Subscriber(position_topic, Marker, poseCallback) 
    time.sleep(4)
    go_arm("go")
    back_arm("go")

    rospy.spin()
