#!/usr/bin/python

import rospy
import std_msgs
from ackermann_msgs.msg import AckermannDrive
import swri_rospy

import serial
import time

class Node:

    def __init__(self):

        # Parameters
	self.ACKERMANN_COMMAND = rospy.get_param("~ackermann_command", "ackermann")
        self.DEV = rospy.get_param("~dev", "/dev/ttyACM1")
        self.BAUD = int(rospy.get_param("~baud", 9600))

        ser = serial.Serial( self.DEV, self.BAUD)
        #ser = serial.Serial( "/dev/ttyACM0", 115200)
        print("connected to: " + ser.portstr)
        rospy.init_node('Starting jrk_node')
        rospy.on_shutdown(self.shutdown)

        # Subscriber
        self.ackermann_subscriber = rospy.Publisher(self.ACKERMANN_COMMAND, AckermannDrive, queue_size="2")
       
    def run(self):
        rospy.loginfo("running jrk_node")
        rospy.spin()


    def handle_jrk_targets(AckermannDrive):#function name*
        target=-1*int(AckermannDrive.steering_angle*2500+2048)
        rospy.loginfo("target: %d", target)
        print('target: %s' % target)
        lowByte = (target & ord("\x1F")) | ord("\xC0")
        highByte = (target >> 5) & ord("\x7F")
        print("about to write", lowByte, highByte)
        ser.write(chr(lowByte))
        ser.write(chr(highByte))

    def shutdown(self):
        rospy.logwarn("Shutting down")

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.logware("Exiting")   
