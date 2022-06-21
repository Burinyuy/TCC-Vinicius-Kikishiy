#!/usr/bin/env python

import rospy
from moveit_python import *
from actionlib_msgs.msg import *
import tf
import time
import math
import numpy as np
from geometry_msgs.msg import PoseStamped,PointStamped,Point,Twist, PoseWithCovarianceStamped
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import actionlib
from actionlib_msgs.msg import GoalID
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal,JointTolerance,PointHeadAction,PointHeadGoal

rospy.init_node('tiago_client_py')

x_c=0
y_c=0
z_c=0
xp=[0,0]
yp=[0,0]
zp=[0,0]
zp_up=[0,0]
xp_leave=[0,0]
yaw=0
quaternion_callback=[0,0,0,0]
dinner_table = [2.7,1.5,0]
purchase_table = [0.95,1.5,270]
sink = [0.6,4.1,90]
counter = [1.4,4.1,90] 
home = [2.35,0.8,90]
xp_leave = [0,0]

def cylinderCallback(pose_message):
    global x_c,y_c,z_c
    #recebe a posicao do frame dos cilindros
    x_c=pose_message.pose.position.x
    y_c=pose_message.pose.position.y
    z_c=pose_message.pose.position.z

def poseCallback(pose_message):
    global yaw,quaternion_callback
    #recebe a orientação do robo em quaternios
    quaternion_callback[0] = pose_message.pose.pose.orientation.x
    quaternion_callback[1] = pose_message.pose.pose.orientation.y
    quaternion_callback[2] = pose_message.pose.pose.orientation.z
    quaternion_callback[3] = pose_message.pose.pose.orientation.w
    #transformação de quaternios para euler
    rpy=tf.transformations.euler_from_quaternion(quaternion_callback)
    yaw=rpy[2]

def move_to_goal(velocity_publisher,xGoal_world,yGoal_world,yawGoal_world):
        print("yawGoalWorld = ", yawGoal_world)
 #transformacao homogenea da posicao em relacao ao ambiente para em relacao ao mapa
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
       
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)#cria um client para a acao move_base 

        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
                rospy.loginfo("Waiting for the move_base action server to come up")

        goal = MoveBaseGoal() 
   

        goal.target_pose.header.frame_id = "map" 
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)#define a posicao desejad
   
        
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yawGoal)#transforma a rotacao de Euler para quaternios
        goal.target_pose.pose.orientation.x = quaternion[0]#define a rotacao desejada
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
                rotate(velocity_publisher,8,yawGoal)
           
                return True

        else:
                rospy.loginfo("The robot failed to reach the destination")
                return False

def rotate(velocity_publisher, angular_speed, desired_angle):

    relative_angle_radians = desired_angle - yaw
 
    angular_speed_degree = angular_speed #define a velocidade angular
    velocity_message = Twist()
 
    angular_speed=math.radians(abs(angular_speed_degree))
    if (relative_angle_radians < 0 ):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    loop_rate = rospy.Rate(1000)  #determina um loop de 1000 vezes por segundo   
    t0 = rospy.Time.now().to_sec() #define o tempo inicial
    relative_angle_degree = math.degrees(abs(relative_angle_radians))
   
    while True :#inicia a rotação do robo
        velocity_publisher.publish(velocity_message) #publica a velocidade
        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree #obtem a posicao angular atual
        print("currente angle", current_angle_degree)
        print("relative angle", relative_angle_degree)
        loop_rate.sleep() 

        if  (current_angle_degree>=relative_angle_degree): #compara a posicao angular relativa e a desejada 
            rospy.loginfo("reached")
            break
   
    velocity_message.angular.z =0 #para o robo
    velocity_publisher.publish(velocity_message)
    print("posicao atual",math.degrees(yaw))

def move_arm(x,y,z,roll,pitch,yaw,side,wait):
    print("start moving")   
    arm_goal = PoseStamped() #cria uma mensagem do tipo PoseStamped
    quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)#transformaçao de Euler para quaternio

    arm_goal.header.frame_id="base_footprint" #define a base do robo como frame de coordenada fixa
    arm_goal.header.stamp = rospy.Time.now() 
    arm_goal.pose.position.x = x #define a posição desejada
    arm_goal.pose.position.y = y
    arm_goal.pose.position.z = z
    arm_goal.pose.orientation.x=quaternion[0] #define a orientação em quaternios
    arm_goal.pose.orientation.y=quaternion[1]
    arm_goal.pose.orientation.z=quaternion[2]
    arm_goal.pose.orientation.w = quaternion[3]
    group= MoveGroupInterface("arm_"+side, "base_footprint") 
    group.moveToPose(arm_goal,"gripper_"+side+"_grasping_frame",wait=wait) 
    print("Move "+side+" arm") 

def gripper(state,side):
    client=actionlib.SimpleActionClient('/gripper_'+side+'_controller/follow_joint_trajectory',FollowJointTrajectoryAction) #Cria o Client
    client.wait_for_server() #espera a confirmação do Client

    goal=FollowJointTrajectoryGoal() #Mensagem para definir o objetivo
    traj=JointTrajectory() #Mensagem da trajetoria
    point=JointTrajectoryPoint() #Mensagem do ponto desejado
    path_tol=JointTolerance() #Mensagem da tolerancia do percurso  
    goal_tol=JointTolerance() #Mensagem da tolerancia do objetivo
    header=Header() #Definicao do resto das mensagens
    header.seq=1
    header.stamp=rospy.Time.now()
    header.frame_id=''

    path_tol.name='path' #definição de  tolerancias baixas
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
    traj.joint_names=["gripper_"+side+"_left_finger_joint", "gripper_"+side+"_right_finger_joint"] #nome das juntas a serem movimentadas

    if state=="open": #parametros para abrir a garra
        point.positions=[0.044,0.044]
    elif state=="close":
        point.positions=[0.033,0.033]#0.01,0.01
    else:
        print("ERROR")
    
    point.velocities=[] #informações para se mover a garra
    point.accelerations=[]
    point.time_from_start.secs=1
    point.time_from_start.nsecs=0

    traj.points=[point] 
    goal.trajectory=traj
    client.send_goal(goal) #envia a mensagem
    wait=client.wait_for_result()
    print("Gripper done") 

def move_torso(state):

    client=actionlib.SimpleActionClient('/torso_controller/follow_joint_trajectory',FollowJointTrajectoryAction) #Cria um client
    client.wait_for_server()

    torso_goal=FollowJointTrajectoryGoal()# mensagem para definir o objetivo
    torso_traj=JointTrajectory() #mensagem da trajetoria
    torso_point=JointTrajectoryPoint() #mensagem do ponto desejado
    torso_path_tol=JointTolerance() #mesnagem da tolerancia do percurso
    torso_goal_tol=JointTolerance() #mensagem da tolerancia do objetivo
    torso_header=Header() #definição do resto das mensagens
    torso_header.seq=1
    torso_header.stamp=rospy.Time.now()
    torso_header.frame_id=''

    torso_path_tol.name='path' #baixas tolerancias
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

    if state=="up": #parametros para se abrir ou fechar a junta
        torso_point.positions=[0.3] #0.34
    elif state=="down":
        torso_point.positions=[0.01]
    else:
        print("ERROR")
    
    torso_point.velocities=[] #informações necessarias para mover o torso
    torso_point.accelerations=[]
    torso_point.time_from_start.secs=1
    torso_point.time_from_start.nsecs=0

    torso_traj.points=[torso_point] 
    torso_goal.trajectory=torso_traj
    client.send_goal(torso_goal) #envia as mensagens
    wait=client.wait_for_result()
    wait=client.wait_for_result()
    print("Moved torso")

def move_head(direction):

    client=actionlib.SimpleActionClient('/head_controller/point_head_action',PointHeadAction) #client do tipo PointHead
    client.wait_for_server()#confirma se o client esta correto

    head_goal = PointHeadGoal() #mensagem do tipo PointHead
    head_point=PointStamped() #mensagem do tipo Point
    head_point.header.stamp=rospy.Time.now() 
    head_point.header.frame_id="/xtion_rgb_optical_frame" 
    
    if direction=="down": #ponto definido em funcao de onde se deseja virar
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
    head_goal.min_duration.secs=1 #parametros do movimento
    head_goal.min_duration.nsecs=0
    head_goal.max_velocity=0.25
    
    client.send_goal(head_goal)
    wait=client.wait_for_result()
    print("Moved head")

def pick(velocity_publisher, start):
    global xc,yc,zc,i,xp,yp,zp,zc_up
    move_torso("up")
    move_head("down")
    
    for i in range(2):
        print("posicao do cilindro x: ", x_c,", y: ",y_c,", z:",z_c)
        time.sleep(5) #aguarda para o recebimento da mensagem
        if start=="go" and z_c!=0:
            xc=x_c-0.021 #
            yc=y_c
            zc=z_c
            zc_up=1.15
            #salva informações para realizar o place
            zp[i]=zc
            zp_up[i]=zc_up
            print("posicao recebida x: ", xc,", y: ",yc,", z:",zc) #posição enviada com destino da gara do robo
            
            if i==2: #encerra o Pick caso o segundo objeto esteja do mesmo lado do primeiro
                if (yp[0]>0 and yp[1]>0) or(yp[0]<0 and yp[1]<0):
                    break
           
            if(yc>0):
                side = "left" #define o braço a ser movimentado
                gripper("open",side)     #pontos adicionais da movimentação do braço        
                move_arm(0.035, 0.596, 1.465, -2.61799, -1.48353, -0.610865,side,True)
                move_arm(0.529, 0.138, zc_up, 3.14159, 0, 0,side,True)
            else:
                side = "right" #define o braço a ser movimentado
                gripper("open",side)   #pontos adicionais da movimentação do braço 
                move_arm(0.035, -0.596, 1.465, -2.61799, -1.48353, -0.610865,side,True)
                move_arm(0.529, -0.138, zc_up, 3.14159, 0, 0,side,True)


            move_arm(0.65, yc, zc, 3.14159, 0, 0,side,True)
            move_arm(xc, yc, zc, 3.14159, 0, 0,side,True)#move o braço ate a posição recebida

            gripper("close",side)#fecha a garra
            move_arm(xc, yc, zc_up,  3.14159, 0, 0,side,True) #pontos adicionais da movimentação do braço 

            if(yc>0):
                move_arm(0.529, 0.138, zc_up,  3.14159, 0, 0,side,True) #posição final do braço após o Pick
            else:
               move_arm(0.529, -0.138, zc_up,  3.14159, 0, 0,side,True)    
    else:
        print("Completed successfully")
        move_head("up")#levanta a cabeça do robo
        move_to_goal(velocity_publisher,counter[0],counter[1],counter[2]) #define e posição do próximo percurso
        place("go") #inicia o Place

def place(start):
    global xp_leave
    move_head("down")
    for i in range(2):
        if start=="go":
            print("Initiating Place")
            if i==2:
                if (yp[0]>0 and yp[1]>0) or(yp[0]<0 and yp[1]<0): #encerra o Place caso só exista um objeto com o TIAGo++
                    break  
            if(yp[i]>0):
                side = "left" #define o braço a ser movimentado
                move_arm(0.529, 0.138, 1.15,  3.14159, 0, 0,side,True) #Verifica se o braço não mudou de posição durante o percurso
                #define as posições do place
                xp[i]=0.818
                yp[i]=0.085

            else:
                side = "right" #define o braço a ser movimentado
                move_arm(0.529, -0.138, 1.15,  3.14159, 0, 0,side,True) #Verifica se o braço não mudou de posição durante o percurso
                #define as posições do place
                xp[i]=0.818
                yp[i]=-0.085  

            #pontos intermediarios
            move_arm(xp[i], yp[i], 1.15, 3.14159, 0, 0,side,True)
            move_arm(xp[i], yp[i], zp[i], 3.14159, 0, 0,side,True)

            gripper("open",side) #abre a garra

            #pontos intermediarios
            move_arm(xp[i], yp[i], zp[i], 3.14159, 0, 0,side,True)
            move_arm(0.6, yp[i], zp[i], 3.14159, 0, 0,side,True)
            
            #fecha a garra
            if(yp[i]>0):
                move_arm(0.529, 0.138, 1.15,  3.14159, 0, 0,side,True)           
            else:
                move_arm(0.529, -0.138, 1.15,  3.14159, 0, 0,side,True)                   
    else:
        print("Completed successfully")
        move_head("up")
        move_to_goal(velocity_publisher,home[0],home[1],home[2])#define e posição do próximo percurso
        print("First task finnished")

def task(start):
    global dinner_table, purchase_table, sink, counter, home
    if start=="go":
        print('start operation')
        move_to_goal(velocity_publisher,purchase_table[0],purchase_table[1],purchase_table[2]) #define a primeira posicao do percurso
        pick(velocity_publisher,"go") #inicia o Pick
    
if __name__ == '__main__':
    try:
        cmd_vel_topic='/mobile_base_controller/cmd_vel' #topic da velocidade
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10) #Publica a velocidade
    
        position_topic = "/robot_pose" #topic da posição do robo
        pose_subscriber = rospy.Subscriber(position_topic, PoseWithCovarianceStamped, poseCallback) #Subscreve a posição

        position_topic = "/cylinder_detector/marker" #topic da posição do cilindro
        pose_subscriber = rospy.Subscriber(position_topic, Marker, cylinderCallback) #subscreve a posição do cilindro

        print('start operation') 
        task("go") #inicia a operação
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")   

