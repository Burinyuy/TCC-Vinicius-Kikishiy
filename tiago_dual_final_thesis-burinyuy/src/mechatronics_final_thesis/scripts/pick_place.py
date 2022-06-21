#!/usr/bin/env python

import rospy
from moveit_python import *
import tf
import time
from geometry_msgs.msg import PoseStamped,PointStamped
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import actionlib
from actionlib_msgs.msg import GoalID
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal,JointTolerance,PointHeadAction,PointHeadGoal


rospy.init_node('tiago_client_py')

x_c=0
y_c=0
z_c=0
xp=[0,0]
yp=[0,0]
zp=[0,0]
zp_up=[0,0]
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

        
def move_arm(x,y,z,roll,pitch,yaw,side,wait):
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
    group= MoveGroupInterface("arm_"+side, "base_footprint") 
    group.moveToPose(arm_goal,"gripper_"+side+"_grasping_frame",wait=wait) 
    print("Move "+side+" arm") 
def gripper(state,side):
    client=actionlib.SimpleActionClient('/gripper_'+side+'_controller/follow_joint_trajectory',FollowJointTrajectoryAction) 
    client.wait_for_server()

    goal=FollowJointTrajectoryGoal()
    traj=JointTrajectory() 
    point=JointTrajectoryPoint() 
    path_tol=JointTolerance() 
    goal_tol=JointTolerance()
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
    traj.joint_names=["gripper_"+side+"_left_finger_joint", "gripper_"+side+"_right_finger_joint"] # Especificamos que  -

    if state=="open": # Segun el parametro del usuario abrimos o cerramos
        point.positions=[0.044,0.044]
    elif state=="close":
        point.positions=[0.033,0.033]#0.01,0.01
    else:
        print("ERROR")
    
    point.velocities=[] # Informacion no necesaria para este movimiento
    point.accelerations=[]
    point.time_from_start.secs=1
    point.time_from_start.nsecs=0

    traj.points=[point] # Encapsulamos los mensajes
    goal.trajectory=traj
    client.send_goal(goal) # Enviamos el mensaje final
    wait=client.wait_for_result()
    print("Gripper done") #Confirmamos por consola

def move_torso(state):

    client=actionlib.SimpleActionClient('/torso_controller/follow_joint_trajectory',FollowJointTrajectoryAction) #Creamos  -un cliente del tipo PointHead
    client.wait_for_server()# Esperamos que el servidor confirme que el cliente es correcto
    
    torso_goal=FollowJointTrajectoryGoal()# Mensaje para definir el objetivo final
    torso_traj=JointTrajectory() # Mensaje que contiene la trayectoria
    torso_point=JointTrajectoryPoint() # Mensaje que contiene el punto deseado
    torso_path_tol=JointTolerance() # Mensaje para la tolerancia del camino
    torso_goal_tol=JointTolerance() # Mensaje para la tolerancia del objetivo
    torso_header=Header() # Definimos un encabezado para usar en el resto de mensajes
    torso_header.seq=1
    torso_header.stamp=rospy.Time.now()
    torso_header.frame_id=''

    torso_path_tol.name='path'# Definimos tolerancias bajas
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
    torso_traj.joint_names=["torso_lift_joint"] # Especificamos que  -

    if state=="up": # Segun el parametro del usuario abrimos o cerramos
        torso_point.positions=[0.3] #0.34
    elif state=="down":
        torso_point.positions=[0.01]
    else:
        print("ERROR")
    
    torso_point.velocities=[] # Informacion no necesaria para este movimiento
    torso_point.accelerations=[]
    torso_point.time_from_start.secs=1
    torso_point.time_from_start.nsecs=0

    torso_traj.points=[torso_point] # Encapsulamos los mensajes
    torso_goal.trajectory=torso_traj
    client.send_goal(torso_goal) # Enviamos el mensaje final
    wait=client.wait_for_result()
    wait=client.wait_for_result()
    print("Moved torso")

def move_head(direction):

    client=actionlib.SimpleActionClient('/head_controller/point_head_action',PointHeadAction) #Creamos  -un cliente del tipo PointHead
    client.wait_for_server()# Esperamos que el servidor confirme que el cliente es correcto
    
    head_goal = PointHeadGoal() # Creamos un mensaje de tipo PointHead
    head_point=PointStamped() # Creamos un mensaje de tipo Point
    head_point.header.stamp=rospy.Time.now() #Definimos el encabezado del Point
    head_point.header.frame_id="/xtion_rgb_optical_frame" #Frame captado por la camara RGB
    
    if direction=="down": # En funcion de la direccion a la que queremos mirar se define el punto
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
    head_goal.pointing_axis.x=0 # Eje sobre el que queremos apuntar la cabeza
    head_goal.pointing_axis.y=0
    head_goal.pointing_axis.z=1
    head_goal.pointing_frame="/xtion_rgb_optical_frame" # Frame de la camara
    head_goal.min_duration.secs=1 #Parametros del movimiento
    head_goal.min_duration.nsecs=0
    head_goal.max_velocity=0.25
    
    client.send_goal(head_goal)
    wait=client.wait_for_result()
    print("Moved head")

def pick(start):
    global xc,yc,zc,i,xp,yp,zp,zc_up

    move_torso("up")
    move_head("down")

    for i in range(2):
        time.sleep(5)
        if start=="go" and z_c!=0:
            xc=x_c-0.021
            yc=y_c
            zc=z_c
            zc_up=1.15#z_c+0.08
            xp[i]=xc
            yp[i]=yc
            zp[i]=zc
            zp_up[i]=zc_up
            print("posicao recebida x: ", xc,", y: ",yc,", z:",zc) 
            
            if i==2:
                if (yp[0]>0 and yp[1]>0) or(yp[0]<0 and yp[1]<0):
                    break
           
            if(yc>0):
                side = "left"               
                move_arm(0.035, 0.596, 1.465, -2.61799, -1.48353, -0.610865,side,True)
                move_arm(0.529, 0.138, zc_up, 3.14159, 0, 0,side,True)
            else:
                side = "right"
                move_arm(0.035, -0.596, 1.465, -2.61799, -1.48353, -0.610865,side,True)
                move_arm(0.529, -0.138, zc_up, 3.14159, 0, 0,side,True)
            gripper("open",side)
            move_arm(0.65, yc, zc, 3.14159, 0, 0,side,True)
            move_arm(xc, yc, zc, 3.14159, 0, 0,side,True)
            gripper("close",side)
            move_arm(xc, yc, zc_up,  3.14159, 0, 0,side,True)
            if(yc>0):
                move_arm(0.529, 0.138, zc_up,  3.14159, 0, 0,side,True)
            else:
               move_arm(0.529, -0.138, zc_up,  3.14159, 0, 0,side,True)       
    else:
        print("Completed successfully")
        move_head("up")

def place(start):
    global xp_leave
    xp_leave = [0,0]
    move_head("down")
    for i in range(2):
        if start=="go":
            print("lets go")
            if i==2:
                if (yp[0]>0 and yp[1]>0) or(yp[0]<0 and yp[1]<0):
                    break  
            if(yp[i]>0):
                side = "left"
                move_arm(0.529, 0.138, 1.15,  3.14159, 0, 0,side,True)
                xp[i]=0.818
                yp[i]=0.085
                move_arm(0.378, 0.138, zp_up[i], 1.5707, 0, 0,side,True)
            else:
                side = "right"
                xp[i]=0.818
                yp[i]=-0.085  
                move_arm(0.529, -0.138, 1.15,  3.14159, 0, 0,side,True)             
                
            move_arm(xp[i], yp[i], 1.15, 3.14159, 0, 0,side,True)
            move_arm(xp[i], yp[i], zp[i], 3.14159, 0, 0,side,True)

            gripper("open",side)
            xp_leave[i]=0.6

            move_arm(xp_leave[i], yp[i], 1.15, 3.14159, 0, 0,side,True)
            
            if(yp[i]>0):
                move_arm(0.529, 0.138, 1.15,  3.14159, 0, 0,side,True)
             
            else:
                move_arm(0.529, -0.138, 1.15,  3.14159, 0, 0,side,True)
              
    else:
        print("Completed successfully")
        move_head("up")

     
if __name__ == '__main__':
    try:
            position_topic = "/cylinder_detector/marker"
            pose_subscriber = rospy.Subscriber(position_topic, Marker, poseCallback) 
            pick("go")
            place("go")




    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")   

