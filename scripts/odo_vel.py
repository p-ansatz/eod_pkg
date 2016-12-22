#!/usr/bin/env python

import rospy
import math
from eod_pkg.msg import Ticks
from eod_pkg.msg import Posa
from eod_pkg.msg import Wheels_Vel
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf


class Odometry():

	def __init__(self, frequency):

		rospy.init_node("eod/odometry")

		# Parametri Fisici
		self.l = rospy.get_param('eod/physics/l',0.1) # Distanza tra ruote
		self.r = rospy.get_param('eod/physics/r',0.1) # Raggio ruota
		self.N = rospy.get_param('eod/hs_interface/n_tick',20) # Numero tick encoder
		self.step = (2.0*pi*self.r)/self.N # Passo encoder

		# Wheels 
		self.rv, self.lv =0.0
		self.read=False	#booleana

		# EOD Twist
		self.x = 0.0
		self.y = 0.0
		self.th = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.vth = 0.0

		#Time
		self.current_time, self.last_time = rospy.Time.now()
		self.rate = rospy.Rate(frequency)

		# Subscribing
		rospy.Subscriber('eod/wheels_vel', Wheels_Vel, self.vel_callback)
		
		# Publishing
		self.odom_pub = rospy.Publisher('eod/odom', Odometry, queue_size=10)
		self.odom_broadcaster = tf.TransformBroadCaster()

	def spin(self):
		
		while not rospy.is_shutdown():
			#self.read = False

			#publishing messages over tf
			odom_trans = TransformStamped()
			odom_trans.header.stamp = current_time
			odom_trans.header.frame_id = 'odom'
			odom_trans.child_frame_id = 'base_link'

			odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
			odom_trans.transform.translation.x = self.x
    		odom_trans.transform.translation.y = self.y
    		odom_trans.transform.translation.z = 0.0
    		odom_trans.transform.rotation = odom_quat

    		self.odom_broadcaster.sendTransform(odom_trans)

    		
    		# publishing the odometry message over ROS
    		odom = Odometry()
    		odom.header.stamp = current_time
    		odom.header.frame_id = 'odom'
    		odom.child_frame_id = 'base_link'

			odom.pose.pose.position.x = self.x
    		odom.pose.pose.position.y = self.y
    		odom.pose.pose.position.z = 0.0
    		odom.pose.pose.orientation = odom_quat;

    		odom.twist.twist.linear.x = self.vx
    		odom.twist.twist.linear.y = self.vy
    		odom.twist.twist.angular.z = self.vth

    		self.odom_pub.publish(odom)

    		#update
    		
    		self.rate.sleep()


	def vel_callback(self, wheels_vel):
		
		#self.read = True
		self.rv = wheels_vel.right
		self.lv = wheels_vel.left
		self.computeOdometry()



	def computeOdometry(self):

		dt = (current_time-last_time).to_sec()	
		

		self.last_time=self.current_time

	


	# def tick_callback(self, tick):
	# 	nr = tick.dx
	# 	nl = tick.sx

	# 	if nr == nl:
	# 		self.x = self.x - self.step*cos(self.theta)
	# 		self.y = self.y + self.step*sin(self.theta)
	# 		#self.theta = self.theta
	# 	else:
	# 		k1 = ((self.l/2.0)*(nr+nl))/(nr-nl)
	# 		k2 = (self.step/self.l)*(nr-nl)

	# 		self.x = self.x - k1*sin(self.theta) + k1*sin(self.theta+k2)
	# 		self.y = self.y + k1*cos(self.theta) - k1*cos(self.theta+k2)
	# 		self.theta = self.theta + k2

	# 	posa = Posa()
	# 	posa.x = self.x
	# 	posa.y = self.y
	# 	posa.theta = self.theta

	# 	pub_posa.publish(posa)


if __name__=='__main__':
	odom = Odometry()
	odom.spin()