#!/usr/bin/env python

import rospy
import serial
import math
import time
import sys
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from arduino_driver.msg import motor_vel

print("Connecting to Arduino...")
ser = serial.Serial('/dev/ttyACM0',9600, timeout=0.4);
ser.reset_input_buffer()
ser.reset_output_buffer()
time.sleep(1)
print("Arduino Connected")

cmd_enbl = None
rate = None

def motorVelocity(m1,m2):
	motorV= [m1,m2]
	ser.write(("v %d %d \r" %(motorV[0],motorV[1])).encode())
	
def stop_cmd():
	motorVelocity(0,0)
	motorVelocity(0,0)
	motorVelocity(0,0)
	motorVelocity(0,0)
	print("\r Stopped Motors \r")

def callback(data):
	vx = data.linear.x
	vy = data.linear.y
	vt = data.angular.z

	rospy.loginfo(rospy.get_caller_id() + ' vx: %f vy: %f vt: %f', vx, vy, vt)

	r = 0.18
	l = 0.51
	qppr = 370

	wL = int(((vx/(r*math.cos(math.atan2(vy,vx))))-(l*vt/r)) * qppr/(2*math.pi*r))
	wR = int(((vx/(r*math.cos(math.atan2(vy,vx))))+(l*vt/r)) * qppr/(2*math.pi*r))

	print("wLeft RPM: " +str(wL))
	print("wRight RPM: " +str(wR))

	if (cmd_enbl):
		motorVelocity(wR,wL)

	rate.sleep()
		
	
    

def listener():
	global cmd_enbl, rate
		
	rospy.init_node('sub_node', anonymous=True)
	cmd_enbl = rospy.get_param('~cmd_enbl', True)
	rate = rospy.Rate(10)
	rospy.Subscriber('/cmd_vel', Twist, callback)
	rospy.on_shutdown(stop_cmd)
	rospy.spin()

if __name__ == '__main__':
    listener()

