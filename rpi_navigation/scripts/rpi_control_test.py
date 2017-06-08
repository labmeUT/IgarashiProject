#!/usr/bin/env python

import rospy
import serial
import sys
import time
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def sendbot(new_cmd):
    rightMotor = 0
    leftMotor = 0
    right_value1 = 0
    right_value2 = 0
    left_value1 = 0
    left_value2 = 0

    if(new_cmd.linear.x == 0):
        power = int(10 * new_cmd.angular.z)
        if(new_cmd.angular.z > 0):
            rightMotor=format((1560 + power) * 4, 'b').zfill(14)
            right_value1=int(rightMotor[0:7],2)
            right_value2=int(rightMotor[7:14],2)

            leftMotor=format((1460 - power) * 4, 'b').zfill(14)
            left_value1=int(leftMotor[0:7],2)
            left_value2=int(leftMotor[7:14],2)
        else:
            rightMotor=format((1560 - power) * 4, 'b').zfill(14)
            right_value1=int(rightMotor[0:7],2)
            right_value2=int(rightMotor[7:14],2)

            leftMotor=format((1460 + power) * 4, 'b').zfill(14)
            left_value1=int(leftMotor[0:7],2)
            left_value2=int(leftMotor[7:14],2)            
    else:
        power = int(10 * new_cmd.linear.x)
        if(new_cmd.angular.z > 0):
            rightMotor = format((1560 + power)*4, 'b').zfill(14)
            right_value1=int(rightMotor[0:7],2)
            right_value2=int(rightMotor[7:14],2)

            leftMotor = format((1560 + power * int(1 - math.fabs(new_cmd.angular.z)))*4, 'b').zfill(14)
            left_value1=int(leftMotor[0:7],2)
            left_value2=int(leftMotor[7:14],2)
        else:
            rightMotor = format((1560 + power * int(1 - math.fabs(new_cmd.angular.z)))*4, 'b').zfill(14)
            right_value1=int(rightMotor[0:7],2)
            right_value2=int(rightMotor[7:14],2)

            leftMotor = format((1560 + power)*4, 'b').zfill(14)
            left_value1=int(leftMotor[0:7],2)
            left_value2=int(leftMotor[7:14],2)

    print "power=", power
    print "rightMotorPulse=", int(rightMotor,2)/4
    print right_value1, right_value2
    print
    print "leftMotorPulse=", int(leftMotor,2)/4
    print left_value1, left_value2
   
    time.sleep(0.1)

def shutdown():
    rospy.loginfo("pololu controller was terminated")
    ser.close()

def listener():
    rospy.init_node('rpi_control_test')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('cmd_vel', Twist, sendbot) # define Twist callback function
    rospy.spin()
    
# main
if __name__ == '__main__':
    listener()
