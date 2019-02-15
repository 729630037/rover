#!/usr/bin/env python

'''
rover_ros.py - Receive sensor values from Launchpad board and publish as topics

'''

#Python client library for ROS
import rospy
import roslib
import sys
import time
import math
import gpiozero as gz
import RPi.GPIO as GPIO
import pigpio 
#Importing ROS data types
from std_msgs.msg import Int16,Int32, Int64, Float32, String, Header, UInt64

pi = pigpio.pi()
  

class Motor(object):
	
	def __init__(self):
		rospy.init_node('rover_ros')
		self.pin= [13,19,26,17,27,22]
		for pins in self.pin:
			pi.set_mode(pins,pigpio.OUTPUT)
		pi.set_PWM_frequency(self.pin[0],50)
		pi.set_PWM_frequency(self.pin[3],50)
		self._left_motor_speed = rospy.Subscriber('left_wheel_speed',Float32,self._Update_Left_Speed)
		self._right_motor_speed = rospy.Subscriber('right_wheel_speed',Float32,self._Update_Right_Speed)
#		self._left_wheel_speed = rospy.Subscriber('lwheel_vel',Float32,self._Left_Speed)
#		self._right_wheel_speed = rospy.Subscriber('rwheel_vel',Float32,self._Right_Speed)

	def _Update_Left_Speed(self, left_speed):
#		self._left_wheel_speed_ = max(min(left_speed.data,1.4),-1.4)	
		self._left_wheel_speed_ =left_speed.data		
		#rospy.loginfo(left_speed.data)
		if self._left_wheel_speed_>=0:
			pi.write(self.pin[1],0)
			pi.write(self.pin[2],1)
			pi.set_PWM_dutycycle(self.pin[0],int(self._left_wheel_speed_))
		else:
			pi.write(self.pin[1],1)
			pi.write(self.pin[2],0)
			pi.set_PWM_dutycycle(self.pin[0],int(abs(self._left_wheel_speed_)))	

	def _Update_Right_Speed(self, right_speed):
#		self._right_wheel_speed_ = max(min(right_speed.data,1.4),-1.4)	
		self._right_wheel_speed_ =right_speed.data		
		#rospy.loginfo(right_speed.data)
		if self._right_wheel_speed_>=0:
			pi.write(self.pin[4],1)
			pi.write(self.pin[5],0)
			pi.set_PWM_dutycycle(self.pin[3],int(self._right_wheel_speed_))
		else:
			pi.write(self.pin[4],0)
			pi.write(self.pin[5],1)
			pi.set_PWM_dutycycle(self.pin[3],int(abs(self._right_wheel_speed_)))

	def stop(self):
		while rospy.is_shutdown():		
			for pins in self.pin:
				pi.write(pins,0)
#			GPIO.cleanup()
'''	
def _Right_Speed(self,rwheelvel):
			rospy.loginfo( rwheelvel.data)

	def _Left_Speed(self,lwheelvel):
			rospy.loginfo( rwheelvel.data)	
'''
if __name__ =='__main__':
#	r = rospy.Rate(30)	
	while not rospy.is_shutdown():	
		motor = Motor()
		rospy.spin()
	motor.stop()


#######################################################################################################################



