#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

import time

info_estado_bumper = False

def mensaje_bumper(estado_bumper):
    global info_estado_bumper
    info_estado_bumper = estado_bumper.data

    #rospy.loginfo(info_estado_bumper)

def movimiento_base():
    rospy.init_node('movimiento_base')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('bumper', Bool, mensaje_bumper)

    rospy.loginfo(info_estado_bumper)
    

    while info_estado_bumper == False:
        rospy.loginfo('Se mueve')
	velocidades = Twist()
	velocidades.linear.x = 0.05
	velocidades.linear.y = 0.0
	velocidades.linear.z = 0.0
	velocidades.angular.x = 0.0
	velocidades.angular.y = 0.0
	velocidades.angular.z = 0.0
	pub.publish(velocidades)
    if info_estado_bumper == True:
        rospy.loginfo('Parado')
	velocidades = Twist()
	velocidades.linear.x = 0.0
	velocidades.linear.y = 0.0
	velocidades.linear.z = 0.0
	velocidades.angular.x = 0.0
	velocidades.angular.y = 0.0
	velocidades.angular.z = 0.0
	pub.publish(velocidades)

    rospy.spin()


if __name__ == '__main__':
    movimiento_base()

