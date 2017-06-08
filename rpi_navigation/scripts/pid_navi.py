# -*- coding: utf-8 -*-

import numpy as np
import rospy

from tf.transformations import euler_from_quaternion

way_x = [-3, -3, 3,  3]    # waypoint
way_y = [-3,  3, 3, -3]    # waypoint

kp = 0.5
ki = 0.01
kd = 0.1

imu_euler = [0, 0, 0] # imu euler angle
gnss_euler = [0, 0, 0] # gnss yaw euler angle

def proportional(x, y, way_x, way_y, angle, kp):
    target_angle = np.arctan2(way_y - y, way_x - x)
    if way_x -x > 0 and way_y - y < 0:
        target_angle = target_angle
    elif way_x -x < 0 and way_y - y < 0:
        target_angle = target_angle
    elif way_x -x > 0 and way_y - y > 0:
        target_angle = 2 * np.pi + target_angle

    deviation = target_angle - angle
    if deviation <= -np.pi:
        deviation = 2 * np.pi + deviation
    change_angle = deviation * kp

    return change_angle

def integral(pre_angle, angle, target_angle, ki):
    if pre_angle == 0:
        return 0

    deviation1 = target_angle - angle
    deviation2 = target_angle - pre_angle

    if deviation1 <= -np.pi:
        deviation1 = 2 * np.pi + deviation1
    if deviation2 <= -np.pi:
        deviation2 = 2 * np.pi + deviation2
    
    return (deviation1 + deviation2) * ki

def diffrential(pre_angle, angle, target_angle, kd):
    if pre_angle == 0:
        return 0

    deviation1 = target_angle - angle
    deviation2 = target_angle - pre_angle
    if deviation1 <= -np.pi:
        deviation1 = 2 * np.pi + deviation1
    if deviation2 <= -np.pi:
        deviation2 = 2 * np.pi + deviation2

    return (deviation2 - deviation1) * kd

def imu_global(imu_msg):
    global e

    q = [0,0,0,0]
    q[0] = imu_msg.orientation.x
    q[1] = imu_msg.orientation.y
    q[2] = imu_msg.orientation.z
    q[3] = imu_msg.orientation.w

    e = euler_from_quaternion((q[0],q[1],q[2],q[3]))    

def gnss_yaw_global(gnss_yaw_msg)
    

def control():
    x = -4                   # start point
    y = -4                   # start point
    angle = np.pi/2          # start angle, 90 degrees
    pre_angle = 0
    accel = 0.15

    while True:
        target_angle = np.arctan2(way_y[i] - y, way_x[i] - x)
  
        prop = proportional(x, y, way_x[i], way_y[i], angle, kp)
        inte = integral(pre_angle, angle, target_angle, ki)
        angle = angle + prop

        diff = diffrential(pre_angle, angle, target_angle, kd)
        angle = angle + inte + diff

        x += accel * np.cos(angle)
        y += accel * np.sin(angle)
        pre_angle = angle
            
        if pow(way_x[i] - x, 2) < 0.1 and pow(way_y[i] - y, 2) < 0.01:
            print(i)
            break
 
def listener():
    rospy.init_node('pid_control')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('/utm', Odometry, control) # define callback function
    rospy.Subscriber('/imu/data' Imu, imu_global)
    rospy.Subscriber('/gnss_yaw' Imu, gnss_yaw_global)
    rospy.spin()    

if __name__ == "__main__":
    listener()
