#!/usr/bin/env python

import rospy
import serial
import sys
import time
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def write_pololu(channel, value1, value2):
    command = bytearray(6)
    command[0] = 0xAA  # Start byte
    command[1] = 0x0C  # Write
    command[2] = 0x04
    command[3] = channel
    command[4] = value1    
    command[5] = value2

    ser.flushInput() 
    ser.write(command)
    ser.flush()

def stop_pololu():
    command = bytearray(1)
    command[0] = 0xA2  # stop byte
    ser.flushInput() 
    ser.write(command)
    ser.flush()

def sendbot(new_cmd):
    rightMotor = 0
    leftMotor = 0
    right_value1 = 0
    right_value2 = 0
    left_value1 = 0
    left_value2 = 0

    if(new_cmd.linear.x == 0):
        power = int(10 * new_cmd.angular.z)
        rightMotor=format((1560 + power) * 4, 'b').zfill(14)
        right_value1=int(rightMotor[0:7],2)
        right_value2=int(rightMotor[7:14],2)

        leftMotor=format((1460 - power) * 4, 'b').zfill(14)
        left_value1=int(leftMotor[0:7],2)
        left_value2=int(leftMotor[7:14],2)
        write_pololu(0x00, right_value1, right_value2) #right motor
        write_pololu(0x01, left_value1, left_value2) #left motor
    else:
        power = int(10 * new_cmd.linear.x)
        if(new_cmd.angular.z > 0):
            rightMotor = format((1560 + power)*4, 'b').zfill(14)
            right_value1=int(rightMotor[0:7],2)
            right_value2=int(rightMotor[7:14],2)

            leftMotor = format((1560 + power * int(1 - math.fabs(new_cmd.angular.z)))*4, 'b').zfill(14)
            left_value1=int(leftMotor[0:7],2)
            left_value2=int(leftMotor[7:14],2)
            write_pololu(0x00, right_value1, right_value2) #right motor
            write_pololu(0x01, left_value1, left_value2) #left motor
        else:
            rightMotor = format((1560 + power * int(1 - math.fabs(new_cmd.angular.z)))*4, 'b').zfill(14)
            right_value1=int(rightMotor[0:7],2)
            right_value2=int(rightMotor[7:14],2)

            leftMotor = format((1560 + power)*4, 'b').zfill(14)
            left_value1=int(leftMotor[0:7],2)
            left_value2=int(leftMotor[7:14],2)
            write_pololu(0x00, right_value1, right_value2) #right motor
            write_pololu(0x01, left_value1, left_value2) #left motor 
   
    rospy.sleep(0.1)
    stop_pololu()

def shutdown():
    rospy.loginfo("pololu controller was terminated")
    ser.close()

def listener():
    rospy.init_node('rpi_control')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('cmd_vel', Twist, sendbot) # define Twist callback function
    rospy.spin()
    
# main
if __name__ == '__main__':
    try:
        # init serial port
        ser=serial.Serial(
            port = '/dev/ttyACM_pololu',
            baudrate = 38400,
            timeout = 0.05,
            writeTimeout = 0.05,
        )
    except serial.serialutil.SerialException:
        rospy.logerr("port not found")
        sys.exit(0)
    listener()
