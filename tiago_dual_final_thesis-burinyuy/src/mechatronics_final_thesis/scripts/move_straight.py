#!/usr/bin/env python

from distutils.file_util import move_file
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time

x=0
y=0


def poseCallback(pose_message):
    global x,y
    x=pose_message.pose.pose.position.x
    y=pose_message.pose.pose.position.y

    print("posicao atual",x,y)

def move(velocity_publisher, distance, is_forward):
        speed=0.5
        #declare a Twist message to send velocity commands
        velocity_message = Twist()
        #get current location 
        global x, y
        x0=x
        y0=y
        if (is_forward):
            velocity_message.linear.x =abs(speed)
        else:
        	velocity_message.linear.x =-abs(speed)
        distance_moved = 0.0
        loop_rate = rospy.Rate(1000) 
       
        while True :
 
                velocity_publisher.publish(velocity_message)
                loop_rate.sleep()
                distance_moved = abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
                print  (distance_moved)               
                if  not (distance_moved<distance):
                    rospy.loginfo("reached")
                    velocity_message.linear.x =0
                    break
        velocity_message.linear.x =0
        velocity_publisher.publish(velocity_message)



if __name__ == '__main__':
    try:
        
        rospy.init_node('tiago_client_py', anonymous=True)

        cmd_vel_topic='/mobile_base_controller/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/robot_pose"
        pose_subscriber = rospy.Subscriber(position_topic, PoseWithCovarianceStamped, poseCallback) 
        time.sleep(2)
        
        move(velocity_publisher,0.036,False)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")