#!/usr/bin/env python3
# license removed for brevity
import rospy
import serial
from std_msgs.msg import String
arduino = serial.Serial(port='/dev/ttyACM0',baudrate=115200,timeout=.1)

x = 0.00
y = 0.00
z = 0.00
w = 0.00
 
def serial_read():
    data = arduino.readline()
    return data.decode("utf-8")
while True:
    value = serial_read()
    t = value.split(': ')

    if t[0] == "Quaternion":
        x = float(t[1].split(", ")[0])
        y = float(t[1].split(", ")[1])
        z = float(t[1].split(", ")[2])
        w = float(t[1].split(", ")[3])
    
    print(x)
    print(y)
    print(z)
    print(w)

# def talker():
#     rospy.init_node('robot_pub_node', anonymous=True)
#     pub = rospy.Publisher('robot_pub', String, queue_size=10)


#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass 