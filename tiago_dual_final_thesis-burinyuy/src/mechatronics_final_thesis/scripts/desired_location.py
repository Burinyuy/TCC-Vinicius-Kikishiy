#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time
import tf
yaw=0
quaternion = [0,0,0,0]

def poseCallback(pose_message):
    global yaw,quaternion
    
    quaternion[0] = pose_message.pose.pose.orientation.x
    quaternion[1] = pose_message.pose.pose.orientation.y
    quaternion[2] = pose_message.pose.pose.orientation.z
    quaternion[3] = pose_message.pose.pose.orientation.w
    rpy=tf.transformations.euler_from_quaternion(quaternion)
    yaw=rpy[2]
    print("posicao atual",math.degrees(yaw))

def rotate (velocity_publisher, desired_angle):
    relative_angle_radians = math.radians(desired_angle) - yaw
    angular_speed_degree = 120
    velocity_message = Twist()
 
    angular_speed=math.radians(abs(angular_speed_degree))
    if relative_angle_radians < 0:
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    loop_rate = rospy.Rate(1000) 
    t0 = rospy.Time.now().to_sec()
    relative_angle_degree = math.degrees(abs(relative_angle_radians))
    while True :
        rospy.loginfo("TIAGo++ rotates")
        velocity_publisher.publish(velocity_message)
        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep() 

        if  (current_angle_degree>=relative_angle_degree):
            rospy.loginfo("reached")
            break
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)


if __name__ == '__main__':
    try:
        
        rospy.init_node('tiago_client_py', anonymous=True)
        cmd_vel_topic='/mobile_base_controller/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/robot_pose"
        pose_subscriber = rospy.Subscriber(position_topic, PoseWithCovarianceStamped, poseCallback) 
        time.sleep(2)
        
        rotate(velocity_publisher,180)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")