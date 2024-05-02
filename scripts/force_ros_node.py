#!/usr/bin/env python

#Libraries
import roslib
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import serial

#Configure serial port, write the port where you've connected the Micro
serial_port = serial.Serial('/dev/ttyUSBO', 115200)

#Node function read serial port info
def NodeControl():
    rospy.init_node('force_ros_node')
    force_pub = rospy.Publisher('Force', Twist, queue_size=10)
    rate = rospy.Rate(100) #Publish frecuency
    

    while not rospy.is_shutdown():
        if serial_port.in_waiting > 0:

            #Read a serial port line
            line = serial_port.readline().strip()

            #Process the line 'X:forcex,Y:forcey,Z:forcez'
            msg_parts = line.split(',')
            force_msg = Twist()
            for force in msg_parts:
                axis, force_value = force.split(':')
                value = float(force_value)
                if axis == 'X':
                    force_msg.linear.x = value
                elif axis == 'Y':
                    force_msg.linear.y = value
                elif axis == 'Z':
                    force_msg.linear.z = value                

            #Publish force_msg
            force_pub.publish(force_msg)
            print("force in x:", force_msg.linear.x)
            print("force in y:", force_msg.linear.y)
            print("force in z:", force_msg.linear.z)

        rate.sleep()

#Main 
if __name__ == '__main__':
    try:
        NodeControl()
    except rospy.ROSInterruptException:
        pass