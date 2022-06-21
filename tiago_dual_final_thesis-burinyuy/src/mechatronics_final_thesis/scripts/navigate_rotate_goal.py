#!/usr/bin/env python  
import rospy
import actionlib
import tf
import math
import numpy as np
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point,Twist, PoseWithCovarianceStamped

yaw=0
quaternion_callback = [0,0,0,0]

def poseCallback(pose_message):
    global yaw,quaternion_callback
    
    quaternion_callback[0] = pose_message.pose.pose.orientation.x
    quaternion_callback[1] = pose_message.pose.pose.orientation.y
    quaternion_callback[2] = pose_message.pose.pose.orientation.z
    quaternion_callback[3] = pose_message.pose.pose.orientation.w
    rpy=tf.transformations.euler_from_quaternion(quaternion_callback)
    yaw=rpy[2]



def move_to_goal(velocity_publisher,xGoal_world,yGoal_world,yawGoal_world):
        print("yawGoalWorld = ", yawGoal_world)

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
        print("yawGoal = ", math.degrees(yawGoal))
        
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
        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
                rospy.loginfo("You have reached the destination ")
                print(quaternion)
                print(xGoal)
                print(yGoal)
                print("adjusting angle")
                rotate(velocity_publisher,yawGoal)

                return True

        else:
                rospy.loginfo("The robot failed to reach the destination")
                return False


def rotate(velocity_publisher, desired_angle):
    relative_angle_radians = desired_angle - yaw
   angular_speed_degree = 8
    velocity_message = Twist()
 
    angular_speed=math.radians(abs(angular_speed_degree))
    if (relative_angle_radians < 0 ):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    loop_rate = rospy.Rate(1000)   
    t0 = rospy.Time.now().to_sec()
    relative_angle_degree = math.degrees(abs(relative_angle_radians))
    while True :

        velocity_publisher.publish(velocity_message)
        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        print("currente angle", current_angle_degree)
        print("relative angle", relative_angle_degree)
        loop_rate.sleep() 

        if  (current_angle_degree>=relative_angle_degree):
            rospy.loginfo("reached")
            break

    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)
    print("posicao atual",math.degrees(yaw))

if __name__ == '__main__':
    try:
        rospy.init_node('tiago_client_py', anonymous=False)

        cmd_vel_topic='/mobile_base_controller/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/robot_pose"
        pose_subscriber = rospy.Subscriber(position_topic, PoseWithCovarianceStamped, poseCallback) 
        time.sleep(2)

        dinner_table = [2.7,1.5,0]
        purchase_table = [0.95,1.5,270]
        sink = [0.6,4.1,90]
        counter = [1.4,4.1,90]
        home = [2.35,0.8,90]
        print('start go to goal')

        move_to_goal(velocity_publisher,dinner_table[0],dinner_table[1],dinner_table[2])

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
