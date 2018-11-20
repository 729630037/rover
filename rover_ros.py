#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import roslib; roslib.load_manifest('mrobot_teleop')
import gpiozero as gz

motor_r = gz.Motor(17,18)
motor_l = gz.Motor(27,22)

class Rover():
	def forward(self,r_speed,l_speed):
		motor_r.forward(r_speed)
		motor_l.forward(l_speed)
		
	def back(self,r_speed,l_speed):
		motor_r.backward(r_speed)
		motor_l.backward(l_speed)
		
	def right(self,r_speed,l_speed):
		motor_r.forward(r_speed)
		motor_l.backward(l_speed)
		
	def left(self,r_speed,l_speed):
		motor_l.forward(r_speed)
		motor_r.backward(l_speed)
		
	def stop(self):
		motor_r.stop()
		motor_l.stop()

rover=Rover()

def twist_speed(x,z):
	if z==0:
		if (x>0 and x<=1) :
			rover.forward(x,x)
#			rospy.loginfo("forward x%f"%(x))
		elif (x>=-1 and x<0) :
			rover.back(-x,-x)
#			rospy.loginfo("backward x%f"%(x))
		else:
			rover.stop()
#			rospy.loginfo("stop x%f"%(x))
	elif z>0 and z<=1:
		rover.left(z,z)
#		rospy.loginfo("left z%f"%(z))
	elif z<0 and z>=-1:
		rover.right(-z,-z)		
#		rospy.loginfo("right z%f"%(z))
	else:
		rover.stop()
#		rospy.loginfo("stop x%f"%(x))
				 
	
		
def callback(msg):
	rotation=msg.angular.z
	twist_x=msg.linear.x
	twist_speed(twist_x,rotation)
#	rospy.loginfo("it is%f,%f"%(msg.linear.x,msg.angular.z))



rospy.init_node('rover')
rospy.Subscriber("/cmd_vel",Twist,callback)
rospy.spin()


