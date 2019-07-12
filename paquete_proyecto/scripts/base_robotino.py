#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool

info_bumper = False
distancia_1 = 0.0

def callback_bumper(mensaje_bumper):
    global info_bumper
    info_bumper = mensaje_bumper.data

def manipulador_mensaje(mensaje_pointcloud):
    global distancia_1
    distancia_1 = mensaje_pointcloud.points[0].x


def robotino():

    rospy.init_node('robotino', anonymous=True)
    sub_bumper = rospy.Subscriber('bumper', Bool, callback_bumper)
    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    sub_distance = rospy.Subscriber ('distance_sensors', PointCloud, manipulador_mensaje)

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

    pub.publish(trajectory)

    tiempo = 0
    tiempo_1 = rospy.Time.now().to_sec()
    while (tiempo <= 6):
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
    while(current_distance <= 1.9) and (info_bumper == False):
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

    while(current_angle <= 1.45):
        velocity_publisher.publish(vel_msg)
        t2=rospy.Time.now().to_sec()
        current_angle= vel_msg.angular.z*(t2-t0)

    vel_msg.angular.z = 0.0
    velocity_publisher.publish(vel_msg)
    rospy.sleep(1)
    vel_msg.linear.x = 0.1
    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    while(current_distance <= 0.5) and (info_bumper == False) and (distancia_1 > 10.0):
        velocity_publisher.publish(vel_msg)
        t3=rospy.Time.now().to_sec()
        current_distance= vel_msg.linear.x*(t3-t0)

    rospy.sleep(2)

    trajectory_1 = JointTrajectory()
    trajectory_1.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper_revolute_joint"]
    trajectory_1.header.stamp = rospy.Time.now()
    trajectory_1.header.frame_id = "base_footprint";

    #Giro de la pinza
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.0, -0.951, 0.434, 1.502, -1.5186, 0.0]
    jtp.time_from_start = rospy.Duration(3)
    trajectory_1.points.append(jtp)
    #Aproximacion1
    jtp_2 = JointTrajectoryPoint()
    jtp_2.positions = [0.0, 0.431, 0.9342, -1.3883, -1.5186, 0.0]
    jtp_2.time_from_start = rospy.Duration(6)
    trajectory_1.points.append(jtp_2)
    #Aproximacion2
    jtp_3 = JointTrajectoryPoint()
    jtp_3.positions = [0.0, 0.5676, 0.5568, -1.1888, -1.5186, 0.0]
    jtp_3.time_from_start = rospy.Duration(9)
    trajectory_1.points.append(jtp_3)
    #Cierre pinza
    jtp_4 = JointTrajectoryPoint()
    jtp_4.positions = [0.0, 0.5676, 0.5568, -1.1888, -1.5186, 2.3]
    jtp_4.time_from_start = rospy.Duration(12)
    trajectory_1.points.append(jtp_4)
    #Cierre pinza2
    jtp_5 = JointTrajectoryPoint()
    jtp_5.positions = [0.0, 0.431, 0.9342, -1.3883, -1.5186, 2.3]
    jtp_5.time_from_start = rospy.Duration(15)
    trajectory_1.points.append(jtp_5)
    #Pulled back con giro de pieza
    jtp_6 = JointTrajectoryPoint()
    jtp_6.positions = [0.0, -0.951, 0.434, 1.502, -1.5186, 2.3]
    jtp_6.time_from_start = rospy.Duration(18)
    trajectory_1.points.append(jtp_6)

    pub.publish(trajectory_1)


    rospy.sleep(20)

    vel_msg.linear.x = -0.1
    vel_msg.linear.y = 0.1
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0

    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    while(current_distance <= 0.5) and (info_bumper == False):
        velocity_publisher.publish(vel_msg)
        t4=rospy.Time.now().to_sec()
        current_distance= vel_msg.linear.x*(t0-t4)


    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0

    velocity_publisher.publish(vel_msg)

    rospy.sleep(4)

    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.5

    t0 = rospy.Time.now().to_sec()
    current_angle = 0.0

    while(current_angle <= 3.0):
        velocity_publisher.publish(vel_msg)
        t5=rospy.Time.now().to_sec()
        current_angle= vel_msg.angular.z*(t5-t0)

    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0

    velocity_publisher.publish(vel_msg)

    trajectory_2 = JointTrajectory()
    trajectory_2.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper_revolute_joint"]
    trajectory_2.header.stamp = rospy.Time.now()
    trajectory_2.header.frame_id = "base_footprint";

    #Aproximacion3
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.0, 0.8053399136399616, -0.12118448224296566, -0.7516505860639642, -1.5186, 2.3]
    jtp.time_from_start = rospy.Duration(3)
    trajectory_2.points.append(jtp)
    #Aproximacion4
    jtp_1 = JointTrajectoryPoint()
    jtp_1.positions = [0.0, 1.191391745257848, -0.4279806398200939, -0.8114758367915041, -1.5186, 2.3]
    jtp_1.time_from_start = rospy.Duration(8)
    trajectory_2.points.append(jtp_1)
    #Abrir pinza
    jtp_2 = JointTrajectoryPoint()
    jtp_2.positions = [0.0, 1.191391745257848, -0.4279806398200939, -0.8114758367915041, -1.5186, 0.0]
    jtp_2.time_from_start = rospy.Duration(11)
    trajectory_2.points.append(jtp_2)

    #Pulled back
    jtp_3 = JointTrajectoryPoint()
    jtp_3.positions = [0.0, -0.951, 0.434, 1.502, 0.0, 0.0]
    jtp_3.time_from_start = rospy.Duration(14)
    trajectory_2.points.append(jtp_3)

    pub.publish(trajectory_2)

    rospy.sleep(16)


    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.5

    t0 = rospy.Time.now().to_sec()
    current_angle = 0.0

    while(current_angle <= 1.3): 
        velocity_publisher.publish(vel_msg)
        t6=rospy.Time.now().to_sec()
        current_angle= vel_msg.angular.z*(t6-t0)

    rospy.sleep(4)

    vel_msg.linear.x = -0.1
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0

    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    while(current_distance <= 1.3) and (info_bumper == False):
        velocity_publisher.publish(vel_msg)
        t7=rospy.Time.now().to_sec()
        current_distance= vel_msg.linear.x*(t0-t7)


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




