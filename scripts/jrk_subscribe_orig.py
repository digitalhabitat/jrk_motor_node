#!/usr/bin/python

import rospy
import std_msgs
from ackermann_msgs.msg import AckermannDrive

import serial
import time

# Parameters
DEV = rospy.get_param("~dev", "/dev/ttyACM1")
BAUD = int(rospy.get_param("~baud", 9600))
ACKERMANN_COMMAND = rospy.get_param("~ackeramnn_command","ackermann")

ser = serial.Serial( DEV, BAUD)
#ser = serial.Serial( "/dev/ttyACM1", 9600)
#ser = serial.Serial( "/dev/ttyACM0", 115200)
print("connected to: " + ser.portstr)
rospy.loginfo("connected: %s", ser.portstr)

def handle_jrk_targets(AckermannDrive):#function name*
	target=-1*int(AckermannDrive.steering_angle*2500+2048)
        rospy.loginfo("target = %d",target)
	print 'target: %s' % target
	lowByte = (target & ord("\x1F")) | ord("\xC0")
	highByte = (target >> 5) & ord("\x7F")
	print("about to write", lowByte, highByte)
	ser.write(chr(lowByte))
	ser.write(chr(highByte))    

rospy.init_node('jrk_steering_node', anonymous=True)
rospy.Subscriber(ACKERMANN_COMMAND, AckermannDrive, handle_jrk_targets)#subscribing to joy teleop
rospy.spin()
