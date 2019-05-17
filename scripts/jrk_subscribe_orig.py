#!/usr/bin/python

import rospy
import std_msgs
from ackermann_msgs.msg import AckermannDrive

import serial
import time

# TODO this node could be made better, 
# need to call jrk2cmd.. persistent name udev rules not working
import os 

cmd_port = os.popen('jrk2cmd --cmd-port').read()
print(cmd_port.rstrip("\n"))
rospy.loginfo("jrk cmd_port = %s ignore parameter dev", cmd_port)

# Parameters
DEV = rospy.get_param("~dev", "/dev/ttyACM0")
BAUD = int(rospy.get_param("~baud", 9600))
ACKERMANN_COMMAND = rospy.get_param("~ackeramnn_command","ackermann")

ser = serial.Serial( cmd_port.rstrip("\n") , BAUD)
#ser = serial.Serial( "/dev/ttyACM1", 9600)
#ser = serial.Serial( "/dev/ttyACM0", 115200)
print("connected to: " + ser.portstr)
rospy.loginfo("connected: %s", ser.portstr)

def handle_jrk_targets(AckermannDrive):

  # positive steering angle should turn left
  target=int(AckermannDrive.steering_angle*2500+2048)
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
