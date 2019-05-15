#!/usr/bin/python

import rospy
import std_msgs
from ackermann_msgs.msg import AckermannDrive

import serial
import time
ser = serial.Serial( "/dev/ttyACM0", 9600)
#ser = serial.Serial( "/dev/ttyACM0", 115200)
print("connected to: " + ser.portstr)

def handle_jrk_targets(AckermannDrive):#function name*
	target=int(AckermannDrive.steering_angle)+100
	print 'target: %s' % target
	lowByte = (target & ord("\x1F")) | ord("\xC0")
	highByte = (target >> 5) & ord("\x7F")
	print("about to write", lowByte, highByte)
	ser.write(chr(lowByte))
	ser.write(chr(highByte))    

rospy.init_node('jrk_steering_node', anonymous=True)
rospy.Subscriber('ackermann', AckermannDrive, handle_jrk_targets)#subscribing to joy teleop
rospy.spin()
