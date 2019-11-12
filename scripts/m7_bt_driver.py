#!/usr/bin/env python

#ROS Modules
import rospy
from std_msgs.msg import Int32, Int64
from std_msgs.msg import Int32MultiArray

import string
import time
import bluetooth

import signal
import sys
import math

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
	bluetooth_serial_handle.close()
	sys.exit(0)
signal.signal(signal.SIGINT,signal_handler)

def control_send(data):

        global bluetooth_serial_handle

	#Note, here we may need change in one motor speed, now taking both speed as data
	try:

                steer_val = data.data[0] # assuming 2digits:ab
                speed_val = data.data[1] # assuming 3digits:cde

                a=math.floor(steer_val/10)
                b=steer_val-10*a

                c=math.floor(speed_val/100)
                d=math.floor(speed_val/10)-10*c
                e=speed_val-100*c-10*d

	except:
		rospy.logwarn("Found exception in BT driver node")
                steer_val = 0
                speed_val = 0

	send_data = '%d%d%d%d%d' %(int(a),int(b),int(c),int(d),int(e))
	print(send_data)

	try:
		bluetooth_serial_handle.send(str(send_data))
	except:
		rospy.logwarn("Unable to send BL data")
		pass

##########################################################################################

#Subscribers (receive control input from main computer(ROS))

rospy.Subscriber('/set_control', Int32MultiArray, control_send)

##########################################################################################

#Function to connect to BL robot: establish bluetooth connection

def connect():
    global bluetooth_mac
    global bluetooth_serial_handle
    while(True):
        try:
            bluetooth_serial_handle = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            bluetooth_serial_handle.connect((bluetooth_mac, 1))
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
		rospy.loginfo(str(receive_data))
		rospy.sleep(0.05)

	    except bluetooth.btcommon.BluetoothError as error:
		#print "Caught BluetoothError: ", error
		rospy.logerr("Caught Bluetooth Error; error")
		time.sleep(5)
		bluetooth_serial_handle = connect()
		pass

	#bluetooth_serial_handle.close()
