#!/usr/bin/env python

import rospy
import std_msgs
from ackermann_msgs.msg import AckermannDrive
import time
import pigpio
import sys
import random

class Node:

    def __init__(self):
        MIN_WIDTH=1000
        MAX_WIDTH=2000		
        pi = pigpio.pi()		
        if not pi.connected:
            rospy.logfatal("pi not connnected")
            rospy.signal_shutdown("pi not connected")   	
        step = 10
        width = MIN_WIDTH
        rospy.init_node('rpi_servo_test_node', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('ackermann_steering', AckermannDrive, servo_callback)
        
    def run(self):
        # home steering postion
        time.sleep(3) # sleep 1 second
        rospy.logerr("starting node")
        g = 4
        while True:
        
            try:
                pi.set_servo_pulsewidth(4, width)
                width+= step
                time.sleep(0.1)
                if width<MIN_WIDTH or width>MAX_WIDTH:
                    step = -step
                    width += step
                time.sleep(0.1)

    def servo_callback(self, AckermannDrive):
        target=AckermannDrive.steering_angle*10/3.5 +5
        p.ChangeDutyCycle(target)
        print("about to write", target)
        rospy.logerr("target %f", target)

if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except: rospy.ROSInterruptException:
        pass
    rospy.logware("Exiting")
    GPIO.cleanup()
    pi.set_servo_pulsewidth(g, 0)
    pi.stop()
