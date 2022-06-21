#!/usr/bin/env python

import rospy
from moveit_python import *
from std_msgs.msg import Header
import actionlib
from actionlib_msgs.msg import GoalID
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PoseStamped, PointStamped
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal,JointTolerance

rospy.init_node('tiago_client_py')

def move_torso(state):

    client=actionlib.SimpleActionClient('/torso_controller/follow_joint_trajectory',FollowJointTrajectoryAction) 
    client.wait_for_server()

    torso_goal=FollowJointTrajectoryGoal()
    torso_traj=JointTrajectory() 
    torso_point=JointTrajectoryPoint() 
    torso_path_tol=JointTolerance() 
    torso_goal_tol=JointTolerance()
    torso_header=Header() 
    torso_header.seq=1
    torso_header.stamp=rospy.Time.now()
    torso_header.frame_id=''

    torso_path_tol.name='path'
    torso_path_tol.position=0.1
    torso_path_tol.velocity=0.1
    torso_path_tol.acceleration=0.1
    torso_goal.path_tolerance=[torso_path_tol]
    
    torso_goal_tol.name='goal'
    torso_goal_tol.position=0.1
    torso_goal_tol.velocity=0.1
    torso_goal_tol.acceleration=0.1
    torso_goal.goal_tolerance=[torso_goal_tol]

    torso_goal.goal_time_tolerance.secs=0.1
    torso_goal.goal_time_tolerance.nsecs=0
    
    torso_traj.header=torso_header
    torso_traj.joint_names=["torso_lift_joint"] 

    if state=="up": 
        torso_point.positions=[0.25] #0.34
    elif state=="down":
        torso_point.positions=[0.01]
    else:
        print("ERROR")
    
    torso_point.velocities=[] 
    torso_point.accelerations=[]
    torso_point.time_from_start.secs=1
    torso_point.time_from_start.nsecs=0

    torso_traj.points=[torso_point] 
    torso_goal.trajectory=torso_traj
    client.send_goal(torso_goal) 
    wait=client.wait_for_result()
    wait=client.wait_for_result()
    print("Moved torso")
   

def move_head(direction):

    client=actionlib.SimpleActionClient('/head_controller/point_head_action',PointHeadAction) 
    client.wait_for_server()
    
    head_goal = PointHeadGoal()
    head_point=PointStamped() 
    head_point.header.stamp=rospy.Time.now() 
    head_point.header.frame_id="/xtion_rgb_optical_frame" 
    
    if direction=="down": 
        head_point.point.x=0
        head_point.point.y=0.6
    elif direction=="up":
        head_point.point.x=0
        head_point.point.y=-0.6
    elif direction=="right":
        head_point.point.x=0.75
        head_point.point.y=0
    elif direction=="left":
        head_point.point.x=-0.75
        head_point.point.y=0
    else:
        head_point.point.x=0
        head_point.point.y=0
    
    head_point.point.z=1

    head_goal.target=head_point
    head_goal.pointing_axis.x=0 
    head_goal.pointing_axis.y=0
    head_goal.pointing_axis.z=1
    head_goal.pointing_frame="/xtion_rgb_optical_frame" 
    head_goal.min_duration.secs=1 
    head_goal.min_duration.nsecs=0
    head_goal.max_velocity=0.25
    
    client.send_goal(head_goal)
    wait=client.wait_for_result()
    print("Moved head")

if __name__ == '__main__':

   move_torso("up")
   move_head("down")
   rospy.spin()
