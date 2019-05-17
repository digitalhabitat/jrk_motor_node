#!/usr/bin/python

# Uses the pySerial library to send and receive data from a Jrk G2.
#
# NOTE: The Jrk's input mode must be "Serial / I2C / USB".
# NOTE: You might need to change the "port_name =" line below to specify the
# right serial port.

import rospy
import std_msgs
from ackermann_msgs.msg import AckermannDrive
import swri_rospy
import serial
import time
import os

# ROS implementation of example code from Jrk G2 Motor Controller User's Guide
# NOTE: could not get set_target to work.. using original code to as a work around

class Node:

    def __init__(self):
        
        # persistent udev naming rule is not working
        # the work around is to use the manufacture provided jrk2cmd program
        self.port_name = os.popen('jrk2cmd --cmd-port').read()
        self.port_name = self.port_name.rstrip("\n")
        self.baud = int(rospy.get_param("~baud", 9600))
        self.scale = int(rospy.get_param("~scale", 2500))
        self.offset = int(rospy.get_param("~offset", 2048))
        self.device_number = 0
        self.ackermann_topic_name = rospy.get_param("~ackermann_topic_name", "ackermann")

        self.port = serial.Serial(self.port_name, self.baud, timeout=0.1, write_timeout=0.1)

        rospy.init_node("jrk_steering_node")
        rospy.on_shutdown(self.shutdown)

        #rospy.Subscriber(self.ACKERMANN_TOPIC_NAME, AckermannDrive, self.ackermann_callback)#subscribing to joy teleop
        swri_rospy.Subscriber(self.ackermann_topic_name, AckermannDrive, self.ackermann_callback, queue_size=1)
        
        rospy.logdebug("port: %s", self.port_name)
        rospy.logdebug("baud: %d", self.baud)
        rospy.logdebug("scale: %d", self.scale)
        rospy.logdebug("offset: %d", self.offset)
        rospy.logdebug("device number: %s", self.device_number)
        rospy.logdebug("ackermann topic name: %s", self.ackermann_topic_name)
   
   
    def run(self):
        rospy.loginfo("Starting jrk_steering_node")
        rospy.spin()

    def ackermann_callback(self, ackermann):
        rospy.logdebug("Recieved steering angle data %f", ackermann.steering_angle)
        # math: angle*(4096/(pi/2))+2048 
        # 2048 is are zero angle position
        # -pi/4 to +pi/4 are max steering angle postions with 0 to 4096 targets
        target = int(ackermann.steering_angle*self.scale+self.offset)  #2067 was too big
        rospy.logdebug("jrk target = %d", target)
        lowByte = (target & ord("\x1F")) | ord("\xC0")
        highByte = (target >> 5) & ord("\x7F")
        #print("about to write", lowByte, highByte)
        self.port.write(chr(lowByte))
        self.port.write(chr(highByte))        

   
    def send_command(self, cmd, *data_bytes):
        if self.device_number == None:
            header = [cmd] # Compact protocol
        else:
            header = [0xAA, self.device_number, cmd & 0x7F] # Pololu protocol
        self.port.write(bytes(header + list(data_bytes)))
    
    # Sets the target. For more information about what this command does,
    # see the "Set Target" command in the "Command reference" section of
    # the Jrk G2 user's guide.
    def set_target(self, target):
        self.send_command(0xC0 + (target & 0x1F), (target >> 5) & 0x7F)
    
    # Gets one or more variables from the Jrk (without clearing them).
    def get_variables(self, offset, length):
        self.send_command(0xE5, offset, length)
        result = self.port.read(length)
        if len(result) != length:
            raise RuntimeError("Expected to read {} bytes, got {}."
                .format(length, len(result)))
            return bytearray(result)

    # Gets the Target variable from the Jrk.
    def get_target(self):
        b = self.get_variables(0x02, 2)    
        return b[0] + 256 * b[1]

    # Gets the Feedback variable from the Jrk.
    def get_feedback(self):
        b = self.get_variables(0x04, 2)
        return b[0] + 256 * b[1]

    def shutdown(self):
        rospy.logwarn("Shutting down")

if __name__ == "__main__":
    try:
        # Choose the serial port name. If the Jrk is connected directly via USB,
        # you can run "jrk2cmd --cmd-port" to get the right name to use here.
        # Linux USB example: "/dev/ttyACM0"
        # macOS USB example: "/dev/cu.usbmodem001234562"
        # Windows example: "COM6"
        #port_name = os.popen('jrk2cmd --cmd-port').read()


        # Choose the baud rate (bits per second). This does not matter if you are
        # connecting to the Jrk over USB. If you are connecting via the TX and RX
        # lines, this should match the baud rate in the Jrk's serial settings.
        # baud_rate = 9600

        # Change this to a number between 0 and 127 that matches the device number of
        # your Jrk if there are multiple serial devices on the line and you want to
        # use the Pololu Protocol.
        #device_number = None

        #port = serial.Serial(port_name, baud_rate, timeout=0.1, write_timeout=0.1)

        #jrk = JrkG2Serial(port, device_number)

        #feedback = jrk.get_feedback()
        #print("Feedback is {}.".format(feedback))

        #target = jrk.get_target()
        #print("Target is {}.".format(target))

        #new_target = 2248 if target < 2048 else 1848
        #print("Setting target to {}.".format(new_target))
        #jrk.set_target(new_target)
        
        jrk = Node()
        jrk.run()
    except rospy.ROSInterruptException:
        pass
    rospy.logware("Exiting")
