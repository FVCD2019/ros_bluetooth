#!/usr/bin/env python

#ROS Modules
import rospy
from std_msgs.msg import Int16, Int32, Int64
from std_msgs.msg import Int32MultiArray, Int16MultiArray
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionResult
from ackermann_msgs.msg import AckermannDriveStamped

import string
import time
import bluetooth

import signal
import sys
import math


finish_flag = False
stop_flag = False
first_chk = 0
sub_chk = 0
prev_sub_chk = 0
speed_val = 450
steer_val = 50
##########################################################################################
#Bluetooth parameters: address

bluetooth_mac = '98:D3:31:FD:3E:0D'

rospy.init_node('ros_bluetooth_driver',anonymous=True)

rospy.loginfo("Staring ROS-Bluetooth Driver node")

##########################################################################################

#Speed handler, this will send speed to BL
#Send control input to arduino after receive message from subscriber '/set_control'

def signal_handler(signal,frame):
	print('pressed ctrl + c!!!')
	stop()
	bluetooth_serial_handle.close()
	sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

def control_send(data):
	global g_dis
	global finish_flag
	global stop_flag
	global first_chk
	global sub_chk
	global prev_sub_chk
	global steer_val
	global speed_val

	#Note, here we may need change in one motor speed, now taking both speed as data
	try:
		if (finish_flag != True):
			speed_val = data.drive.speed
			steer_val = (-data.drive.steering_angle )  # * -0.5 ~ 0.5

			#if speed_val > 0.4:
			#	speed_val = 0.4
			#if speed_val < -0.4:
			#	speed_val = -0.4

			#if speed_val > 0:
			#	speed_val = 0.35 + abs(steer_val)*0.045
			#else:
			#	speed_val = -0.36 - abs(steer_val)*0.06
			if speed_val > 0.5:
				speed_val = 0.5
			elif speed_val > 0 and speed_val < 0.29 :
				speed_val = 0.29
			elif speed_val < -0.5:
				speed_val = -0.5
			elif speed_val < 0 and speed_val > -0.33 :
				speed_val = -0.33
			speed_val = (speed_val+0.5) * 900
			#if(speed_val < 450):
			#	steer_val = steer_val*-1

			if steer_val < -0.5:
				steer_val = -0.5
			if steer_val > 0.5:
				steer_val = 0.5
			steer_val = (steer_val + 0.5) * 100
		
			#print(first_chk)
			if(first_chk == 1):
				speed_val = 850
	
			if(g_dis < 50):
				speed_val = speed_val * 0.7
					

			print(stop_flag)
			if(stop_flag):
				if(speed_val > 450):
					speed_val = 850
				else:
					speed_val = 68


			print("[steer] : ", int(steer_val), "     [speed] : ", int(speed_val))

			a = math.floor(steer_val/10)
			b = steer_val-10*a
			c = math.floor(speed_val/100)
			d = math.floor(speed_val/10)-10*c
			e = speed_val-100*c-10*d
		else:
			steer_val = 50
			speed_val = 450
			a = math.floor(steer_val/10)
			b = steer_val-10*a
			c = math.floor(speed_val/100)
			d = math.floor(speed_val/10)-10*c
			e = speed_val-100*c-10*d
	except:
		rospy.logwarn("Found exception in BT driver node")
		steer_val = 50
		speed_val = 450
		a = math.floor(steer_val/10)
		b = steer_val-10*a
		c = math.floor(speed_val/100)
		d = math.floor(speed_val/10)-10*c
		e = speed_val-100*c-10*d

	send_data = '%d%d%d%d%d' %(int(a),int(b),int(c),int(d),int(e))
	#print(send_data)

	try:
		bluetooth_serial_handle.send(str(send_data))
		if(first_chk == 1):
			first_chk = first_chk+1
			time.sleep(0.05)
	except:
		rospy.logwarn("Unable to send BL data")
		pass

def stop():
	global bluetooth_serial_handle

	#Note, here we may need change in one motor speed, now taking both speed as data

	steer_val = 50
	speed_val = 450
	a = math.floor(steer_val/10)
	b = steer_val-10*a
	c = math.floor(speed_val/100)
	d = math.floor(speed_val/10)-10*c
	e = speed_val-100*c-10*d

	send_data = '%d%d%d%d%d' %(int(a),int(b),int(c),int(d),int(e))

	try:
		bluetooth_serial_handle.send(str(send_data))
		print("[stop] : ", send_data)
	except:
		rospy.logwarn("Unable to send BL data")
		pass

def result_check(msg):
	global finish_flag
	if msg.status.status == 3:
		finish_flag = True
		for i in range(10):
			stop()
		print("goodbye")
	else:
		finish_flag = False

def chk_stop(msg):
	global stop_flag
	if msg.data < 5:
		stop_flag = True
	else:
		stop_flag = False

def get_gp(msg):
	global g_dis
	g_dis = msg.data

##########################################################################################

#Subscribers (receive control input from main computer(ROS))

#rospy.Subscriber('/cmd_vel', Twist, control_send)
rospy.Subscriber('/ackermann_cmd', AckermannDriveStamped, control_send)
rospy.Subscriber('/move_base/result', MoveBaseActionResult, result_check)
rospy.Subscriber('/car_dis', Int16, chk_stop)
rospy.Subscriber('/goal_dis', Int16, get_gp)

##########################################################################################
#Function to connect to BL robot: establish bluetooth connection

def connect():
	global first_chk
	global bluetooth_mac
	global bluetooth_serial_handle
	while(True):
		try:
			bluetooth_serial_handle = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
			bluetooth_serial_handle.connect((bluetooth_mac, 1))
			first_chk = first_chk+1
			break;
		except bluetooth.btcommon.BluetoothError as error:
			bluetooth_serial_handle.close()
			rospy.logwarn("Unable to connect, Retrying in 10s...")
			#print "Could not connect: ", error, "; Retrying in 10s..."
			time.sleep(10)
	return bluetooth_serial_handle;

bluetooth_serial_handle = connect()

##########################################################################################

#Main code : receive data from arduino

if __name__ == '__main__':

	while(True):
		try:
			receive_data = bluetooth_serial_handle.recv(300)
			#rospy.loginfo(str(receive_data))		
			rospy.sleep(0.05)

		except bluetooth.btcommon.BluetoothError as error:
			#print "Caught BluetoothError: ", error
			rospy.logerr("Caught Bluetooth Error; error")
			time.sleep(5)
			bluetooth_serial_handle = connect()
			pass

	#bluetooth_serial_handle.close()
