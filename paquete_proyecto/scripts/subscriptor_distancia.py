#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud

def manipulador_mensaje(mensaje_pointcloud):
    for i in range(0,9):
        rospy.loginfo('Sensor_Distancia_' + str(i + 1) + ': ' + str(mensaje_pointcloud.points[i].x))
        #Hemos puesto i+1 para contabilizar los sensores tal y como vienen en las imagenes

def subscriptor_distancia():
    rospy.init_node('subscriptor_distancia')
    rospy.Subscriber ('distance_sensors', PointCloud, manipulador_mensaje)

    rospy.spin()

if __name__ == '__main__':
    subscriptor_distancia()
