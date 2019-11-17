#!/usr/bin/env python

#ROS Modules
import rospy
#from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from std_msgs.msg import Int32, Int64
from std_msgs.msg import Float32MultiArray

import string
import time
import bluetooth

import signal
import sys
import math

##########################################################################################

#Bluetooth parameters

#BL Robot's Bluetooth address
bluetooth_mac = '98:D3:31:FD:3E:0D'

rospy.init_node('ros_bluetooth_driver',anonymous=True)

rospy.loginfo("Staring ROS-Bluetooth Driver node")

##########################################################################################

#Speed handler, this will send speed to BL

'''def control_send(data):

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
                steer = 0.0
                speed = 0.0
	
	send_data = '%d%d%d%d%d' %(int(a),int(b),int(c),int(d),int(e))

	try:
		bluetooth_serial_handle.send(str(send_data))
	except:
		rospy.logwarn("Unable to send BL data")
		pass'''
	
##########################################################################################
'''def reset_robot(data):
    	global bluetooth_serial_handle	

	send_data = 'r\r'

        rospy.loginfo("Resetting robot")

	try:
		bluetooth_serial_handle.send(str(send_data))
		rospy.sleep(1)
	except:
		rospy.logwarn("Unable to send BL data")
		pass '''

##########################################################################################
'''
#Publishers
 
comm_check_handle = rospy.Publisher('comm_check', Int32, queue_size=1)
 
#Subscribers

rospy.Subscriber('/set_control', Float32MultiArray, control_send)
rospy.Subscriber('/reset', Int32, reset_robot)'''

##########################################################################################
'''#Key board handler

def quit_code(signum, frame):

	# To reset the robot
	reset_robot(0)

	rospy.loginfo("Quitting code")
	sys.exit(1)

signal.signal(signal.SIGINT, quit_code)'''

##########################################################################################
'''
#Function to publish topics to ROS
def publish_topics(string_list):
        
         global comm_check_handle
        

	#Reading each line in list and spliting it
	for line in string_list:
		
		real_data = line.split(" ")
		#print real_data

                #if(real_data[0] == 's'):
                        #left_speed_handle.publish(float(real_data[1]))
                        #right_speed_handle.publish(float(real_data[2]))

                if(real_data[0] == 's'):
                        comm_check_handle.publish(float(real_data[1]))'''

##########################################################################################
'''
#Function to remove spaces from sentence 

def remove_space(sentence):
	org_sentence = []
	for char in sentence:
		if(char != ""):
			org_sentence.append(char)

	#print org_sentence
	publish_topics(org_sentence)'''
	
##########################################################################################

'''def decode_string(string):
        This function will decode the buffer data 
	param: string, string buffer


	start_flag = 0

	final_string = ""

	string_list = []

	decode_list = list(string)
	string_len = len(decode_list)

	#IMU value extract
        
	for i in range(0,string_len):
		#Start reading when y is found
		if(decode_list[i] == 'i' and (string_len - i) > 10):
			start_flag = 1

		#Read until a carriage return occurs
		if(start_flag == 1):
			if(decode_list[i] != '\n'):
				final_string = final_string + decode_list[i]
			else:
				start_flag = 0
		#If it is a new line, append to a list
		if(decode_list[i] == "\n"):
			string_list.append(final_string)

			final_string = ''

	final_string = remove_space(string_list)



	string_list = []
	final_string  = ""

        #Motor speed value extract
	for i in range(0,string_len):
		#Start reading when y is found
		if(decode_list[i] == 's' and (string_len - i) > 8):
			start_flag = 1

		if(start_flag == 1):
			if(decode_list[i] != '\n'):
				final_string = final_string + decode_list[i]
			else:
				start_flag = 0
		#If it is a new line, append to a list
		if(decode_list[i] == "\n"):
			string_list.append(final_string)

			final_string = ''	


        final_string = remove_space(string_list)'''

##########################################################################################

#Function to connect to BL robot
	
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

#Main code

if __name__ == '__main__':

	while(True):
	    try:
		bluetooth_serial_handle.send(str('01234'))
		receive_data = bluetooth_serial_handle.recv(300)
		rospy.loginfo(str(receive_data))
		 #list_rec = receive_data.split("\n")
		 #list_rec = list(receive_data)
		#Continously receiving data from robot and decode it
		 #decode_string(list_rec)

		rospy.sleep(0.05)

	    except bluetooth.btcommon.BluetoothError as error:
		#print "Caught BluetoothError: ", error
		rospy.logerr("Caught Bluetooth Error; error")
		time.sleep(5)
		bluetooth_serial_handle = connect()
		pass

	bluetooth_serial_handle.close()

