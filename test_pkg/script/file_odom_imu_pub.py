#!/usr/bin/env python3

# dummy speed for cibot

import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import time

import serial
import os

from std_msgs.msg import String
from std_msgs.msg import Int16


from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import MagneticField

imu_data_seq_counter = 0

rospy.init_node('odometry_imu_pub')



odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

# arduino = serial.Serial(port='/dev/ttyACM0',baudrate=115200,timeout=.1)

q_pub = rospy.Publisher('imu/data',Imu,queue_size = 1)
node_name = rospy.get_name()

x_imu = 0.00
y_imu = 0.00
z_imu = 0.00
w_imu = 0.00

encoder = 0
encoder_old = 0


x = 0.0
y = 0.0
th = 0.0

# initialize time
current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(100)

f = open("data1.txt", "r")
data = f.read().split("\n")
print(data)

i=0
l_data=0
def serial_read():
    data = arduino.readline()
    return data.decode("utf-8")

while not rospy.is_shutdown():    
    #update time
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()

    if i > 15: i = 0
    # print(i)
    if i == 0:
        w_imu = float(data[l_data].split("\t")[1])
        x_imu = float(data[l_data].split("\t")[2])
        y_imu = float(data[l_data].split("\t")[3])
        z_imu = float(data[l_data].split("\t")[4])
        encoder = float(data[l_data].split("\t")[0])
        print(w_imu,end=" ")
        print(x_imu,end=" ")
        print(y_imu,end=" ")
        print(z_imu,end=" ")
        print(encoder)
        l_data += 1

    i+=1

    vx = 0
    vy = 0
    vth = 0
    vz = ((encoder - encoder_old)/dt)/200

    print(vz)
    # compute odometry in a typical way given the velocities of the robot

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
    odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))
    odom.pose.covariance = [1e-05, 0, 0, 0, 0, 0,
                        0, 1e-05, 0, 0, 0, 0,
                        0, 0, 1e12, 0, 0, 0,
                        0, 0, 0, 1e12, 0, 0,
                        0, 0, 0, 0, 1e12, 0,
                        0, 0, 0, 0, 0, 0.001]

    # set the velocity
    odom.child_frame_id = "base_footprint_ekf"
    odom.twist.twist = Twist(Vector3(vx, vy, vz), Vector3(0, 0, vth))
    odom.twist.covariance = [0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0]

    # publish the message
    odom_pub.publish(odom)
    encoder_old = encoder
    last_time = current_time
        

    # print(x_imu)
    # print(y_imu)
    # print(z_imu)
    # print(w_imu)


    # print(encoder)

    ########################################

    imu_data = Imu()
        
    # # quaternion = self.bno055.get_quaternion_orientation()
    # # linear_acceleration = self.bno055.get_linear_acceleration()
    # # gyroscope = self.bno055.get_gyroscope()

    imu_data_seq_counter=+1
    imu_data.header.seq = imu_data_seq_counter

    imu_data.header.stamp = rospy.Time.now()
    frame_id = rospy.get_param(node_name + '/frame_id', 'base_footprint_ekf')
    imu_data.header.frame_id = frame_id
    imu_data.header.seq = imu_data_seq_counter

    imu_data.orientation.w = w_imu
    imu_data.orientation.x = x_imu
    imu_data.orientation.y = y_imu
    imu_data.orientation.z = z_imu

    q_pub.publish(imu_data)

    #####################################


    r.sleep()
