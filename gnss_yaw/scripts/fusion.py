#! /usr/bin/env python

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher('/fusion_all', Odometry, queue_size = 10)
fusion_utm = Odometry()

gnss_yaw_data = [0, 0, 0 ,0, 0, 0]     #gnss yaw quaternion
pre_gnss_seq = 0
pre_imu_q = [0, 0, 0, 0]  #previious imu quaternion
fusion_q = [0, 0, 0, 0]    #quaternion
fusion_e = [0, 0, 0]

def utm(utm_msg):
    fusion_utm.pose.pose.position.x = utm_msg.pose.pose.position.x
    fusion_utm.pose.pose.position.y = utm_msg.pose.pose.position.y
    fusion_utm.pose.pose.position.z = utm_msg.pose.pose.position.z

    pub.publish(fusion_utm)

def imu(imu_msg):
    global pre_imu_q, pre_gnss_seq

    # gnss_yaw is absolute orientation. If can't get gnss_yaw
    if pre_gnss_seq == gnss_yaw_data[0]:
        fusion_utm.pose.pose.orientation.x = fusion_q[0] - (imu_msg.orientation.x - pre_imu_q[0])
        fusion_utm.pose.pose.orientation.y = fusion_q[1] - (imu_msg.orientation.y - pre_imu_q[1])
        fusion_utm.pose.pose.orientation.z = fusion_q[2] - (imu_msg.orientation.z - pre_imu_q[2])
        fusion_utm.pose.pose.orientation.w = fusion_q[3] - (imu_msg.orientation.w - pre_imu_q[3])
        
    pre_imu_q[0] = imu_msg.orientation.x
    pre_imu_q[1] = imu_msg.orientation.y
    pre_imu_q[2] = imu_msg.orientation.z
    pre_imu_q[3] = imu_msg.orientation.w

    pre_gnss_seq = gnss_yaw_data[0]

    pub.publish(fusion_utm)

def gnss_yaw(gnss_yaw_msg):
    global gnss_yaw_data    # use gnss_q in other functions

    gnss_yaw_data[0] = gnss_yaw_msg.header.seq
    gnss_yaw_data[1] = gnss_yaw_msg.orientation.x
    gnss_yaw_data[2] = gnss_yaw_msg.orientation.y
    gnss_yaw_data[3] = gnss_yaw_msg.orientation.z
    gnss_yaw_data[4] = gnss_yaw_msg.orientation.w

    fusion_utm.pose.pose.orientation.x = gnss_yaw_msg.orientation.x
    fusion_utm.pose.pose.orientation.y = gnss_yaw_msg.orientation.y
    fusion_utm.pose.pose.orientation.z = gnss_yaw_msg.orientation.z
    fusion_utm.pose.pose.orientation.w = gnss_yaw_msg.orientation.w

    pub.publish(fusion_utm)
    pub.publish(fusion_utm)

def fusion(fusion_msg):
    global fusion_q
    
    fusion_q[0] = fusion_msg.pose.pose.orientation.x
    fusion_q[1] = fusion_msg.pose.pose.orientation.y
    fusion_q[2] = fusion_msg.pose.pose.orientation.z
    fusion_q[3] = fusion_msg.pose.pose.orientation.w

    fusion_e = euler_from_quaternion((fusion_q[0],fusion_q[1],fusion_q[2],fusion_q[3]))
    print fusion_e[2] /np.pi *180

def shutdown():
    rospy.loginfo("gnss_yaw_node was terminated")

def listener():
    rospy.init_node('gnss_imu_fusion_node')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('/imu/data', Imu, imu) # ROS callback function
    rospy.Subscriber('/utm', Odometry, utm) # ROS callback function
    rospy.Subscriber('/gnss_yaw', Imu, gnss_yaw) # ROS callback function
    rospy.Subscriber('/fusion_all', Odometry, fusion) # ROS callback function
    rospy.spin()

if __name__ == '__main__':
    listener()
