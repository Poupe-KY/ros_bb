#!/usr/bin/env python3

# dummy speed for cibot

import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import time

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0

# initialize time
current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10)

while not rospy.is_shutdown():    
    
    vx = 0.12 #m/s
    vy = 0
    vth = 0

    # compute odometry in a typical way given the velocities of the robot
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th)) * dt
    delta_y = (vx * sin(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_footprint_enc",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    odom.pose.covariance = [1e-05, 0, 0, 0, 0, 0,
                          0, 1e-05, 0, 0, 0, 0,
                          0, 0, 1e12, 0, 0, 0,
                          0, 0, 0, 1e12, 0, 0,
                          0, 0, 0, 0, 1e12, 0,
                          0, 0, 0, 0, 0, 0.001]

    # set the velocity
    odom.child_frame_id = "base_footprint_ekf"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    odom.twist.covariance = [0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0]

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
