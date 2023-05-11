#!/usr/bin/env python3


"""
Transform the messaage from the steering feedbaack topic
to a float message that the PID library can suscribe to.

node: remap_fb_node
Subscribe to: /steeringmotors/feedback
Publish to :  /wheel_1_steering_pid/state
              /wheel_2_steering_pid/state
              /wheel_3_steering_pid/state
              /wheel_4_steering_pid/state


by Pablo
Last review: 2022/09/29
"""

import rospy
import numpy as np

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64


motor_1_media_ = [0.0 , 0.0 , 0.0, 0.0 , 0.0]
motor_2_media_ = [0.0 , 0.0 , 0.0, 0.0 , 0.0]
motor_3_media_ = [0.0 , 0.0 , 0.0, 0.0 , 0.0]
motor_4_media_ = [0.0 , 0.0 , 0.0, 0.0 , 0.0]


def radTorpm(radsec):
    motor_rpm = ((radsec *60.0)/(2.0*3.14))
    return motor_rpm


def media_movil(datos, ventana):
    """
    Aplica un filtro de media móvil de ventana 'ventana' a una serie de datos 'datos'.
    """
    ventana = int(ventana)
    # Crea una matriz con una fila por cada punto de datos y una columna por cada punto de la ventana.
    matriz = np.column_stack([datos[i: len(datos) - ventana + i + 1] for i in range(ventana)])
    # Aplica la función de media en cada fila y devuelve la serie suavizada.
    return np.mean(matriz, axis=1)


def talker(msg):
    fb1_msg = Float64()
    fb2_msg = Float64()
    fb3_msg = Float64()
    fb4_msg = Float64()

    motor_1_media_.pop()
    motor_1_media_.append(radTorpm(msg.data[0]))
    fb1_msg.data = media_movil(motor_1_media_, 5)+0.9


    motor_2_media_.pop()
    motor_2_media_.append(-radTorpm(msg.data[1]))
    fb2_msg.data = media_movil(motor_2_media_, 5)+0.9


    motor_3_media_.pop()
    motor_3_media_.append(radTorpm(msg.data[2]))
    fb3_msg.data = media_movil(motor_3_media_, 5)+0.9


    motor_4_media_.pop()
    motor_4_media_.append(-radTorpm(msg.data[3]))
    fb4_msg.data = media_movil(motor_4_media_, 5)+0.9

    fb_wheel_1_pub.publish(fb1_msg)
    fb_wheel_2_pub.publish(fb2_msg)
    fb_wheel_3_pub.publish(fb3_msg)
    fb_wheel_4_pub.publish(fb4_msg)



rospy.init_node('remap_fb_node_driving')
rospy.Subscriber("/driving_motors/feedback/angular/radsec", Float64MultiArray, talker, queue_size=1)

fb_wheel_1_pub = rospy.Publisher('/driving_pid/pid/motor1/state', Float64, queue_size=1)
fb_wheel_2_pub = rospy.Publisher('/driving_pid/pid/motor2/state', Float64, queue_size=1)
fb_wheel_3_pub = rospy.Publisher('/driving_pid/pid/motor3/state', Float64, queue_size=1)
fb_wheel_4_pub = rospy.Publisher('/driving_pid/pid/motor4/state', Float64, queue_size=1)

rospy.spin()

