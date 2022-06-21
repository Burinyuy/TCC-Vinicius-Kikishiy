#!/usr/bin/env python

import rospy
from moveit_python import *
from std_msgs.msg import Header
import actionlib
from actionlib_msgs.msg import GoalID
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal,JointTolerance

rospy.init_node('tiago_client_py')

def gripper(state,side):
    client=actionlib.SimpleActionClient('/gripper_'+side+'_controller/follow_joint_trajectory',FollowJointTrajectoryAction) 

    goal=FollowJointTrajectoryGoal()
    traj=JointTrajectory() 
    point=JointTrajectoryPoint() 
    path_tol=JointTolerance()
    goal_tol=JointTolerance() 
    header=Header() 
    header.seq=1
    header.stamp=rospy.Time.now()
    header.frame_id=''

    path_tol.name='path'
    path_tol.position=0.1
    path_tol.velocity=0.1
    path_tol.acceleration=0.1
    goal.path_tolerance=[path_tol]

    goal_tol.name='goal'
    goal_tol.position=0.1
    goal_tol.velocity=0.1
    goal_tol.acceleration=0.1
    goal.goal_tolerance=[goal_tol]

    goal.goal_time_tolerance.secs=0.1
    goal.goal_time_tolerance.nsecs=0


    traj.header=header
    traj.joint_names=["gripper_"+side+"_left_finger_joint", "gripper_"+side+"_right_finger_joint"] 

    if state=="open": 
        point.positions=[0.044,0.044]
    elif state=="close":
        point.positions=[0.033,0.033]#0.01,0.01
    else:
        print("ERROR")
    
    point.velocities=[] 
    point.time_from_start.secs=1
    point.time_from_start.nsecs=0

    traj.points=[point] 
    goal.trajectory=traj
    client.send_goal(goal) 
    wait=client.wait_for_result()
    print("Gripper done") 
if __name__ == '__main__':
   gripper("open","left")
rospy.spin()
