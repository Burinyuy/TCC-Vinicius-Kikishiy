#!/usr/bin/env python

import rospy
from moveit_python import *
from std_msgs.msg import Header
import actionlib
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PoseStamped, PointStamped

rospy.init_node('tiago_client_py')

def move_head(direction):

    client=actionlib.SimpleActionClient('/head_controller/point_head_action',PointHeadAction) #Creamos  -un cliente del tipo PointHead
    client.wait_for_server()# Esperamos que el servidor confirme que el cliente es correcto
    
    head_goal = PointHeadGoal() # mensagem do  PointHead
    point=PointStamped() # mensagem do tipo Point
    point.header.stamp=rospy.Time.now() 
    point.header.frame_id="/xtion_rgb_optical_frame" #Frame da camera
    
    if direction=="down": # ponto definido em funcao de onde se deseja virar
        point.point.x=0
        point.point.y=0.75
    elif direction=="up":
        point.point.x=0
        point.point.y=-0.75
    elif direction=="right":
        point.point.x=0.75
        point.point.y=0
    elif direction=="left":
        point.point.x=-0.75
        point.point.y=0
    else:
        point.point.x=0
        point.point.y=0
    
    point.point.z=1

    head_goal.target=point
    head_goal.pointing_axis.x=0 # Eje sobre el que queremos apuntar la cabeza
    head_goal.pointing_axis.y=0
    head_goal.pointing_axis.z=1
    head_goal.pointing_frame="/xtion_rgb_optical_frame" 
    head_goal.min_duration.secs=1 #Parametros do movimento
    head_goal.min_duration.nsecs=0
    head_goal.max_velocity=0.25
    
    client.send_goal(head_goal)
    wait=client.wait_for_result()
    print("Moved head")

if __name__ == '__main__':
   #rospy.init_node('map_navigation', anonymous=False)
   #specify the cordinates of the goal location
 
   move_head("down")
   rospy.spin()
