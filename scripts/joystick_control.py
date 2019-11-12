#!/usr/bin/env python
import rospy
import numpy as np
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

class JOY:
	def __init__(self):
		print("init")
		rospy.init_node('joystick_control') #anonymous=True
		self.sub = rospy.Subscriber("/joy", Joy, self.joyCB)
		self.c_pub = rospy.Publisher("set_control", Int32MultiArray, queue_size=1)
		self.control_data = Int32MultiArray()
		self.control_data.data = []

	def joyCB(self, data):
		throttle = data.axes[4]
		steer = -data.axes[3]
		reset_button = data.buttons[7]
		quit_button = data.buttons[6]

		throttle_val = (throttle+1)*450
		steer_val = (steer+1)*72 + 20

		if (reset_button):
			throttle_val = 0
			steer_val = 92

		if (quit_button):
			done = True

		self.control_data.data = [throttle_val, steer_val]
		self.c_pub.publish(self.control_data)

	def do(self):
		rospy.spin()

joy = JOY()
time.sleep(1)
joy.do()
