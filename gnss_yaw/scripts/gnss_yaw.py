#! /usr/bin/env python

# Calculate absolute orientation from trajectory 
# while using GNSS UTM cordinate date

import rospy
import time
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

x = []
y = []

opt_corr = 0.9 # threshold correlation coefficeint
opt_dist = 0.3 # threshold distance

pub_yaw = rospy.Publisher('/gnss_yaw', Imu, queue_size = 10)
imu_msg = Imu()

def gnss_yaw(utm):
    x.append(utm.pose.pose.position.x)    # read ROS /utm topic data
    y.append(utm.pose.pose.position.y)

    if len(x) > 10:
        x.pop(0)
        y.pop(0)

    if len(x) > 9:
        yaw = np.arctan2(y[9]-y[0], x[9]-x[0]) #calculate yaw from trajectory
        corr = np.corrcoef(x, y) #calculate correlation coefficeint(for linearity)
        dist = np.linalg.norm((y[9]-y[0]-(x[9]-x[0]))) #calculate Euclidean distance 

        # If you get optimal distance and correlation coefficeint, 
        # publish ROS messages with GNSS yaw(quaternion)
        if dist > opt_dist and abs(corr[0,1]) > opt_corr:
            imu_msg.header.frame_id = "gnss_yaw"
            imu_msg.header.stamp = rospy.Time.now()
            q = quaternion_from_euler(0, 0, yaw)
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]
            
            pub_yaw.publish(imu_msg)
            #e = euler_from_quaternion((q[0],q[1],q[2],q[3]))
            #print x
            #print y
            print yaw/np.pi *180

def shutdown():
    rospy.loginfo("gnss_yaw_node was terminated")

def listener():
    rospy.init_node('gnss_yaw_node')
    rospy.on_shutdown(shutdown)
    rospy.Subscriber('utm', Odometry, gnss_yaw) # ROS callback function
    rospy.spin()

if __name__ == '__main__':
    listener()
