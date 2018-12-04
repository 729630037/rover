#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import roslib; roslib.load_manifest('ros_erle_rover_navigation')
import gpiozero as gz

motor_r = gz.Motor(17,18)
motor_l = gz.Motor(27,22)
wheel=0.36

class Rover():
	def forward(self,r_speed,l_speed):
		motor_r.forward(r_speed)
		motor_l.forward(l_speed)
		
	def back(self,r_speed,l_speed):
		motor_r.backward(r_speed)
		motor_l.backward(l_speed)

	def right(self,r_speed,l_speed):
		motor_r.forward(r_speed)
		motor_l.forward(l_speed)

	def left(self,r_speed,l_speed):
		motor_r.backward(r_speed)
		motor_l.backward(l_speed)
		
	def ro(self,r_speed,l_speed):
		motor_r.forward(r_speed)
		motor_l.backward(l_speed)
		
	def lo(self,r_speed,l_speed):
		motor_l.forward(r_speed)
		motor_r.backward(l_speed)
		
	def stop(self):
		motor_r.stop()
		motor_l.stop()

rover=Rover()

def twist_speed(x,z):
	x=max(min(x,1),-1)
	z=max(min(z,1),-1)	
	if z==0:
		if x>0:
			rover.forward(0.5,0.5)
#			rospy.loginfo("forward x%f"%(x))
		elif x<0:
			rover.back(0.5,0.5)
#			rospy.loginfo("backward x%f"%(x))
		else:
			rover.stop()
#			rospy.loginfo("stop x%f"%(x))
	elif x==0:
		if z>0 and z<=1:
			rover.lo(0.8,0.8)
	#		rospy.loginfo("left z%f"%(z))
		elif z<0 and z>=-1:
			rover.ro(0.8,0.8)		
	#		rospy.loginfo("right z%f"%(z))
		else:
			rover.stop()
	else:
		right=(z*wheel)/2+x
		left=x*2-right
		if right>0 and left >0:
			rover.right(0.7,1)
		else:
			rover.left(1,0.7)
#		rospy.loginfo("stop x%f"%(x))
				 
	
		
def callback(msg):
	rotation=msg.angular.z
	twist_x=msg.linear.x
	twist_speed(twist_x,rotation)
	rospy.loginfo("it is%f,%f"%(msg.linear.x,msg.angular.z))



rospy.init_node('rover')
rospy.Subscriber("/cmd_vel",Twist,callback)
rospy.spin()


