#! /usr/bin/env python

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

imu_q = []
gnss_yaw_q = []

pub = rospy.Publisher('/fusion_utm', Odometry, queue_size = 10)
fusion_utm = Odometry()

def utm(utm_msg):
    fusion_utm.pose.pose.position.x = utm_msg.pose.pose.position.x
    fusion_utm.pose.pose.position.y = utm_msg.pose.pose.position.y
    fusion_utm.pose.pose.position.z = utm_msg.pose.pose.position.z

    pub.publish(fusion_utm)

def imu(imu_msg):
    fusion_utm.pose.pose.orientation.x = imu_msg.orientation.x
    fusion_utm.pose.pose.orientation.y = imu_msg.orientation.y
    fusion_utm.pose.pose.orientation.z = imu_msg.orientation.z
    fusion_utm.pose.pose.orientation.w = imu_msg.orientation.w

    pub.publish(fusion_utm)

def shutdown():
    rospy.loginfo("gnss_yaw_node was terminated")

def listener():
    rospy.init_node('gnss_imu_fusion_node')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('/imu/data', Imu, imu) # ROS callback function
    rospy.Subscriber('/utm', Odometry, utm) # ROS callback function
    rospy.spin()

if __name__ == '__main__':
    listener()
