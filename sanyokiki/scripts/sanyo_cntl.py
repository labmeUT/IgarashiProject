#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import serial
import sys
import time
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry

StartByte_1 = "S"
StartByte_2 = "T"
ForwardBackward_Neutral = 512 #neutral value
LeftRight_Neutral = 512 #neutral value
EngineSpeed = "0578" #minimam speed
EngineOn = "0000"  #engine off value, engine on value is "0080"
AutonomousOff = "0000"
MaxSpeed_Limit = "1000"
CorrectionDataA = "0000"
CorrectionDataB = "0000"
LineFeed = chr(0x0d)
CarriageReturn = chr(0x0a)
CommandLength = 36 #For example, command 'ST02000200057800800000100000000000' has 36 length words
PowerAdjust = 240

# serial write
def send_sanyocontroller(ForwardBackward, LeftRight, EngineOn):
    ControlCommand = StartByte_1 +StartByte_2 +ForwardBackward +LeftRight +EngineSpeed +EngineOn +AutonomousOff +MaxSpeed_Limit +CorrectionDataA +CorrectionDataB +LineFeed +CarriageReturn

    now = []
    for i in range(CommandLength):
        #t = datetime.datetime.now() #debug option
        #now.append(t.microsecond) #debug option
        ser.flushInput()
        ser.write(ControlCommand[0+i:1+i])
        ser.flush()
        #print ControlCommand[0+i:1+i] #debug option
        time.sleep(0.001) # need 1 msec sleep
    #print ControlCommand #debug option
    #print now #debug option


def sendbot_cmd(new_cmd):
    global ForwardBackward_value
    global LeftRight_value
    Power = 0
    ForwardBackward_value = 0
    LeftRight_value = 0

    if(new_cmd.linear.x == 0):
        Power = int(PowerAdjust * math.fabs(new_cmd.angular.z)) #In any case, PowerAdjust value most change and adjust 
        if(new_cmd.angular.z > 0):
            ForwardBackward_value = ForwardBackward_Neutral
            LeftRight_value = LeftRight_Neutral +Power
        else:
            ForwardBackward_value = ForwardBackward_Neutral
            LeftRight_value = LeftRight_Neutral -Power
    elif(new_cmd.linear.x > 0):
        Power = int(PowerAdjust * math.fabs(new_cmd.linear.x))
        if(new_cmd.angular.z > 0):
            ForwardBackward_value = ForwardBackward_Neutral +Power
            LeftRight_value = LeftRight_Neutral +Power*int(1 - math.fabs(new_cmd.angular.z))
        elif(new_cmd.angular.z < 0):
            ForwardBackward_value = ForwardBackward_Neutral +Power
            LeftRight_value = LeftRight_Neutral +Power*int(1 - math.fabs(new_cmd.angular.z))
        else:
           ForwardBackward_value = ForwardBackward_Neutral +Power
           LeftRight_value = LeftRight_Neutral
    elif(new_cmd.linear.x < 0):
        Power = int(PowerAdjust * math.fabs(new_cmd.linear.x))
        if(new_cmd.angular.z > 0):
            ForwardBackward_value = ForwardBackward_Neutral -Power
            LeftRight_value = LeftRight_Neutral +Power*int(1 - math.fabs(new_cmd.angular.z))
        elif(new_cmd.angular.z < 0):
            ForwardBackward_value = ForwardBackward_Neutral -Power
            LeftRight_value = LeftRight_Neutral +Power*int(1 - math.fabs(new_cmd.angular.z))
        else:
           ForwardBackward_value = ForwardBackward_Neutral -Power
           LeftRight_value = LeftRight_Neutral 

    ForwardBackward_value = '0' +hex(ForwardBackward_value)[2:5]
    LeftRight_value = '0' + hex(LeftRight_value)[2:5]

    #send_sanyocontroller(ForwardBackward_value, LeftRight_value)
    #print "test"

def sendbot(engine):
    global ForwardBackward_value
    global LeftRight_value

    try:
        ForwardBackward_value = ForwardBackward_value
    
    except NameError:
        ForwardBackward_value = '0' +hex(ForwardBackward_Neutral)[2:5]
        LeftRight_value = '0' +hex(LeftRight_Neutral)[2:5] 

    if(engine.data == 80):
        EngineOn = '00' + str(engine.data)
    else:
        EngineOn = '0000'
    send_sanyocontroller(ForwardBackward_value, LeftRight_value, EngineOn)

def shutdown():
    rospy.loginfo("sanyo controller was terminated")
    ser.close()

def listener():
    rospy.init_node('sanyo_controller')
    rospy.on_shutdown(shutdown)
    print "ready to controller"
    rospy.Subscriber('engine_onoff', Int8, sendbot) # define Twist callback function
    rospy.Subscriber('cmd_vel', Twist, sendbot_cmd) #callback function
    rospy.spin()
    
# main
if __name__ == '__main__':
    try:
        # init serial port
        ser=serial.Serial(
            port = '/dev/serial/by-path/platform-3f980000.usb-usb-0:1.3:1.0-port0',
            baudrate = 115200,
            timeout = 0.05,
            writeTimeout = 0.05,
        )
    except serial.serialutil.SerialException:
        rospy.logerr("port not found")
        sys.exit(0)

    listener()
