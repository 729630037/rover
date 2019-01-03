#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import roslib; roslib.load_manifest('rover_bringup')
import gpiozero as gz
import pigpio 
pi = pigpio.pi()

wheel=0.36
#global z0=0

pin= [13,19,26,17,27,22]
for pins in pin:
	pi.set_mode(pins,pigpio.OUTPUT)
pi.set_PWM_frequency(pin[0],50)
pi.set_PWM_frequency(pin[3],50)

def twist_speed(x,z):
	right=(z*wheel)/2+x
	left=x*2-right
	if x!=0:	
		if z>0.1:
			pi.write(pin[1],0)
			pi.write(pin[2],1)
			pi.set_PWM_dutycycle(pin[0],int(200))
			pi.write(pin[4],0)
			pi.write(pin[5],1)
			pi.set_PWM_dutycycle(pin[3],int(200))	
		elif z<-0.1:
			rospy.loginfo("it is%f"%(z))
			pi.write(pin[1],1)
			pi.write(pin[2],0)
			pi.set_PWM_dutycycle(pin[0],int(200))
			pi.write(pin[4],1)
			pi.write(pin[5],0)
			pi.set_PWM_dutycycle(pin[3],int(abs(200)))	

		else:
			pi.write(pin[1],0)
			pi.write(pin[2],1)
			pi.set_PWM_dutycycle(pin[0],int(200))
			pi.write(pin[4],1)
			pi.write(pin[5],0)
			pi.set_PWM_dutycycle(pin[3],200)	

	else:
			pi.write(pin[1],0)
			pi.write(pin[2],0)
			pi.set_PWM_dutycycle(pin[0],int(50))
			pi.write(pin[4],0)
			pi.write(pin[5],0)
			pi.set_PWM_dutycycle(pin[3],int(abs(50)))	
	
		
def callback(msg):
	rotation=msg.angular.z
	twist_x=msg.linear.x
	twist_speed(twist_x,rotation)
#	rospy.loginfo("it is%f,%f"%(msg.linear.x,msg.angular.z))



rospy.init_node('rover')
rospy.Subscriber("/cmd_vel",Twist,callback)
rospy.spin()


