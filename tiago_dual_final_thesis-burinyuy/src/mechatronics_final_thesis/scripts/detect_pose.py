#!/usr/bin/env python  
import rospy
import actionlib
import tf
import math
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

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
if __name__ == '__main__':
    try:
        
        rospy.init_node('tiago_client_py', anonymous=True)

        position_topic = "/cylinder_detector/marker"
        pose_subscriber = rospy.Subscriber(position_topic, Marker, poseCallback) 
        time.sleep(10)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")