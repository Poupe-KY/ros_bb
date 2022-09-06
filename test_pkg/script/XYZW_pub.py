#!/usr/bin/env python3

# from asyncore import read
# from tkinter import W
# from sensor_msgs.msg import Imu
import serial
import rospy
import os
import tf

from std_msgs.msg import String
from std_msgs.msg import Int16


from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import MagneticField
# from std_srvs.srv import Empty, EmptyResponse
# from std_srvs.srv import Trigger, TriggerResponse 

# rospy.init_node('robot_pub_node', anonymous=True)
# pub = rospy.Publisher('robot_pub',Int16, queue_size=10)

imu_data_seq_counter = 0

rospy.init_node('ros_imu_bno055_node', anonymous=False)

pub = rospy.Publisher('robot_pub',String, queue_size=10)

arduino = serial.Serial(port='/dev/ttyACM0',baudrate=115200,timeout=.1)

q_pub = rospy.Publisher('imu/data',Imu,queue_size = 1)

# frame_id = rospy.get_param(node_name + '/frame_id', 'imu_link')

node_name = rospy.get_name()

rate = rospy.Rate(10)

x = 0.00
y = 0.00
z = 0.00
w = 0.00

e = '0'
 
def serial_read():
    data = arduino.readline()
    return data.decode("utf-8")

while not rospy.is_shutdown():  
    value = serial_read()
    t = value.split(': ')
 
    if t[0] == "Quaternion":
        x = float(t[1].split(", ")[0])
        y = float(t[1].split(", ")[1])
        z = float(t[1].split(", ")[2])
        w = float(t[1].split(", ")[3])
    
    if t[0] == "Encoder":
        e = String(t[1].split(", ")[0])
    
    print(x)
    print(y)
    print(z)
    print(w)
    print(e)

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

    imu_data.orientation.w = w
    imu_data.orientation.x = x
    imu_data.orientation.y = y
    imu_data.orientation.z = z

    # print(imu_data)
    q_pub.publish(imu_data) 
    pub.publish(e)
    rate.sleep()


