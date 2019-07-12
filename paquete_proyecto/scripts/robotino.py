#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool

info_bumper = False
orientacion = 0.0
distancia = 0.0

def callback(mensaje):
    global orientacion
    global distancia
    orientacion = mensaje.pose.pose.orientation.z
    distancia = mensaje.pose.pose.position.x

def callback_bumper(mensaje_bumper):
    global info_bumper
    info_bumper = mensaje_bumper.data

def robotino():

    rospy.init_node('robotino', anonymous=True)
    sub = rospy.Subscriber('odom', Odometry, callback)
    sub_bumper = rospy.Subscriber('bumper', Bool, callback_bumper)
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

    rospy.sleep(2)

    trajectory = JointTrajectory()
    trajectory.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper_revolute_joint"]
    trajectory.header.stamp = rospy.Time.now()
    trajectory.header.frame_id = "base_footprint";

    #Pulled back pose
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.0, -0.951, 0.434, 1.502, 0.0, 0.0]
    jtp.time_from_start = rospy.Duration(3)
    trajectory.points.append(jtp)
    #Aproximacion inicial
    jtp_2 = JointTrajectoryPoint()
    jtp_2.positions = [0.0, -0.937, 0.072, 0.853, 0.0, 0.0]
    jtp_2.time_from_start = rospy.Duration(6)
    trajectory.points.append(jtp_2)
    #Aproximacion a la pieza
    jtp_3 = JointTrajectoryPoint()
    jtp_3.positions = [0.0, -0.383, -0.2531, 0.638, 0.0, 0.0]
    jtp_3.time_from_start = rospy.Duration(9)
    trajectory.points.append(jtp_3)
    #Cierra la pinza
    jtp_4 = JointTrajectoryPoint()
    jtp_4.positions = [0.0, -0.383, -0.2531, 0.638, 0.0, 1.7]
    jtp_4.time_from_start = rospy.Duration(12)
    trajectory.points.append(jtp_4)
    #Retroceso
    jtp_5 = JointTrajectoryPoint()
    jtp_5.positions = [0.0, -0.937, 0.072, 0.853, 0.0, 1.7]
    jtp_5.time_from_start = rospy.Duration(15)
    trajectory.points.append(jtp_5)
    #Pulled back pose
    jtp_6 = JointTrajectoryPoint()
    jtp_6.positions = [0.0, -0.951, 0.434, 1.502, 0.0, 1.7]
    jtp_6.time_from_start = rospy.Duration(18)
    trajectory.points.append(jtp_6)   

    pub.publish(trajectory)
 
    tiempo = 0
    tiempo_1 = rospy.Time.now().to_sec()
    while (tiempo <= 2):
        tiempo_2 = rospy.Time.now().to_sec()
        tiempo = tiempo_2-tiempo_1

    vel_msg = Twist()

    vel_msg.linear.x = 0.1
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0

    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    while(current_distance <= 1.9):
        velocity_publisher.publish(vel_msg)
        t1=rospy.Time.now().to_sec()
        current_distance= vel_msg.linear.x*(t1-t0)

    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)

    trajectory_1 = JointTrajectory()
    trajectory_1.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper_revolute_joint"]
    trajectory_1.header.stamp = rospy.Time.now()
    trajectory_1.header.frame_id = "base_footprint";
    #Pulled back pose
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.0, -0.951, 0.434, 1.502, 0.0, 1.7]
    jtp.time_from_start = rospy.Duration(3)
    trajectory_1.points.append(jtp)
    #Aproximacion inicial
    jtp_2 = JointTrajectoryPoint()
    jtp_2.positions = [0.0, -0.937, 0.072, 0.853, 0.0, 1.7]
    jtp_2.time_from_start = rospy.Duration(6)
    trajectory_1.points.append(jtp_2)
    #Aproximacion a la pieza
    jtp_3 = JointTrajectoryPoint()
    jtp_3.positions = [0.0, -0.383, -0.2531, 0.638, 0.0, 1.7]
    jtp_3.time_from_start = rospy.Duration(9)
    trajectory_1.points.append(jtp_3)
    #Cierra la pinza
    jtp_4 = JointTrajectoryPoint()
    jtp_4.positions = [0.0, -0.383, -0.2531, 0.638, 0.0, 0.0]
    jtp_4.time_from_start = rospy.Duration(12)
    trajectory_1.points.append(jtp_4)
    #Retroceso
    jtp_5 = JointTrajectoryPoint()
    jtp_5.positions = [0.0, -0.937, 0.072, 0.853, 0.0, 0.0]
    jtp_5.time_from_start = rospy.Duration(15)
    trajectory_1.points.append(jtp_5)
    #Pulled back pose
    jtp_6 = JointTrajectoryPoint()
    jtp_6.positions = [0.0, -0.951, 0.434, 1.502, 0.0, 0.0]
    jtp_6.time_from_start = rospy.Duration(18)
    trajectory_1.points.append(jtp_6)   

    pub.publish(trajectory_1)

    tiempo = 0
    tiempo_1 = rospy.Time.now().to_sec()
    while (tiempo <= 2):
        tiempo_2 = rospy.Time.now().to_sec()
        tiempo = tiempo_2-tiempo_1
    

    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.5

    t0 = rospy.Time.now().to_sec()
    current_angle = 0.0

    while(current_angle <= math.pi/2): 
        velocity_publisher.publish(vel_msg)
        t2=rospy.Time.now().to_sec()
        current_angle= vel_msg.angular.z*(t2-t0)

    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)
    rospy.sleep(1)
    vel_msg.linear.x = 0.1
    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    while(current_distance <= 0.5):
        velocity_publisher.publish(vel_msg)
        t3=rospy.Time.now().to_sec()
        current_distance= vel_msg.linear.x*(t3-t0)

    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        robotino()
    except rospy.ROSInterruptException:
        pass

