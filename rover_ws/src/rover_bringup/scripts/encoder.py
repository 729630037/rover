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
from numpy import array

#Class to handle serial data from Launchpad and converted to ROS topics


class Encoder(object):

    def __init__(self):
        rospy.init_node('encoder')
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)

        self.levA = 0
        self.levB = 0
        self.lastGpio = None
        self.epinA = rospy.get_param('~pinA')
        self.epinB = rospy.get_param('~pinB')
        self.Encoder_Ticks = 0
        
        ### initialize variables
        self.target = 0
        self.motor = 0
        self.vel = 0
        self.integral = 0
        self.error = 0
        self.derivative = 0
        self.previous_error = 0
        self.wheel_prev = 0
        self.wheel_latest = 0
        self.then = rospy.Time.now()
        self.wheel_mult = 0
        self.prev_encoder = 0
        
        self.Kp = rospy.get_param('~Kp')
        self.Ki = rospy.get_param('~Ki')
        self.Kd = rospy.get_param('~Kd')
        self.out_min = rospy.get_param('~out_min')
        self.out_max = rospy.get_param('~out_max')
        self.rate = rospy.get_param('~rate',50)
        self.rolling_pts = rospy.get_param('~rolling_pts',2)
        self.timeout_ticks = rospy.get_param('~timeout_ticks',4)
        self.ticks_per_meter = rospy.get_param('ticks_meter')
        self.vel_threshold = rospy.get_param('~vel_threshold', 0.001)
        self.encoder_min = rospy.get_param('encoder_min', -2147483648)
        self.encoder_max = rospy.get_param('encoder_max', 2147483648)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        self.prev_vel = [0.0] * self.rolling_pts
        self.wheel_latest = 0.0
        self.prev_pid_time = rospy.Time.now()
        
        GPIO.setmode(GPIO.BCM)
        pi = pigpio.pi()
        
        for pin in self.epinA,self.epinB:
            pi.set_mode(pin, pigpio.INPUT)
            pi.set_pull_up_down(pin, pigpio.PUD_UP)
        self.cb1 = pi.callback(self.epinA, pigpio.EITHER_EDGE, self.encodercallback)			
        self.cb2 = pi.callback(self.epinB, pigpio.EITHER_EDGE, self.encodercallback)
        self.pub_motor = rospy.Publisher('motor_cmd',Float32,queue_size=10) 	
        self.pub_wheel = rospy.Publisher('wheel',Int64,queue_size=10)
        self.pub_vel = rospy.Publisher('wheel_vel', Float32,queue_size=10)
        rospy.Subscriber("wheel_vtarget", Float32, self.targetCallback) 	

    def encodercallback(self,gpio,level,tick):
        if gpio == self.epinA:
            self.levA = level
        else:
            self.levB = level

        if gpio != self.lastGpio: # debounce
            self.lastGpio = gpio
            if   gpio == self.epinA and level == 1:
                if self.levB == 1:
                    self.callback(-1)
            elif gpio == self.epinB and level == 1:
                if self.levA == 1:
                    self.callback(1)
	
    def callback(self,way):
        self.Encoder_Ticks += way
        enc= self.Encoder_Ticks
        if (enc < self.encoder_low_wrap and self.prev_encoder > self.encoder_high_wrap) :
            self.wheel_mult = self.wheel_mult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_encoder < self.encoder_low_wrap) :
            self.wheel_mult = self.wheel_mult - 1
        self.wheel_latest = 1.0 * (enc + self.wheel_mult * (self.encoder_max - self.encoder_min)) / self.ticks_per_meter 
        self.prev_encoder = enc           
 
    def calcVelocity(self):
        self.dt_duration = rospy.Time.now() - self.then
        self.dt = self.dt_duration.to_sec()
        rospy.logdebug("-D- %s caclVelocity dt=%0.3f wheel_latest=%0.3f wheel_prev=%0.3f" % (self.nodename, self.dt, self.wheel_latest, self.wheel_prev))
        
        if (self.wheel_latest == self.wheel_prev):
            # we haven't received an updated wheel lately
            cur_vel = (1 / self.ticks_per_meter) / self.dt    # if we got a tick right now, this would be the velocity
            if abs(cur_vel) < self.vel_threshold: 
                # if the velocity is < threshold, consider our velocity 0
                rospy.logdebug("-D- %s below threshold cur_vel=%0.3f vel=0" % (self.nodename, cur_vel))
                self.appendVel(0)
                self.calcRollingVel()
            else:
                rospy.logdebug("-D- %s above threshold cur_vel=%0.3f" % (self.nodename, cur_vel))
                if abs(cur_vel) < self.vel:
                    rospy.logdebug("-D- %s cur_vel < self.vel" % self.nodename)
                    # we know we're slower than what we're currently publishing as a velocity
                    self.appendVel(cur_vel)
                    self.calcRollingVel()
            
        else:
            # we received a new wheel value
            cur_vel = (self.wheel_latest - self.wheel_prev) / self.dt
            self.appendVel(cur_vel)
            self.calcRollingVel()
            rospy.logdebug("-D- %s **** wheel updated vel=%0.3f **** " % (self.nodename, self.vel))
            self.wheel_prev = self.wheel_latest
            self.then = rospy.Time.now()
            
        self.pub_vel.publish(self.vel)        

    def doPid(self):
    #####################################################
        pid_dt_duration = rospy.Time.now() - self.prev_pid_time
        pid_dt = pid_dt_duration.to_sec()
        self.prev_pid_time = rospy.Time.now()
        self.error = self.target - self.vel
        self.integral = self.integral + (self.error * pid_dt)
        #rospy.loginfo("i = i + (e * dt):  %0.3f = %0.3f + (%0.3f * %0.3f)" % (self.integral, self.integral, self.error, pid_dt))
        self.derivative = (self.error - self.previous_error) / pid_dt
        self.previous_error = self.error
    
        self.motor = self.motor+(self.Kp * self.error) + (self.Ki * self.integral) + (self.Kd * self.derivative)
    	#print self.error,self.integral,self.derivative
        if self.motor > self.out_max:
            self.motor = self.out_max
            self.integral = self.integral - (self.error * pid_dt)
        if self.motor < self.out_min:
            self.motor = self.out_min
            self.integral = self.integral - (self.error * pid_dt)
      
        if (self.target == 0):
            self.motor = 0
        #rospy.loginfo("vel:%0.2f tar:%0.2f err:%0.2f int:%0.2f der:%0.2f ## motor:%d " % (self.vel, self.target, self.error, self.integral, self.derivative, self.motor))

    def appendVel(self, val):
        self.prev_vel.append(val)
        del self.prev_vel[0]

    def calcRollingVel(self):
        p = array(self.prev_vel)
        self.vel = p.mean()

    def targetCallback(self, msg):
        self.target = msg.data
        self.ticks_since_target = 0

    def __del__(self):
        try:
            self.cb1.cancel() 
            self.cb2.cancel() 
        except:
            pass

                
    def spinOnce(self):
        self.previous_error = 0.0
        self.prev_vel = [0.0] * self.rolling_pts
        self.integral = 0.0
        self.error = 0.0
        self.derivative = 0.0 
        self.vel = 0.0
        while not rospy.is_shutdown() :
            self.calcVelocity()
            self.doPid()
            self.pub_motor.publish(self.motor)
            self.pub_wheel.publish(self.Encoder_Ticks)
            self.r.sleep()
            self.ticks_since_target += 1
            if self.ticks_since_target == self.timeout_ticks:
                self.pub_motor.publish(0)
   
    def spin(self):
        self.r = rospy.Rate(self.rate) 
        self.then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
        self.wheel_prev = self.wheel_latest
        self.then = rospy.Time.now()
        while not rospy.is_shutdown():
            self.spinOnce()
            self.r.sleep()


if __name__ =='__main__':
	encoder=Encoder()
	try:
		encoder.spin()
	except rospy.ROSInterruptException:
		rospy.logwarn("Error in main function")


#######################################################################################################################


