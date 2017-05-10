#!/usr/bin/env python

import rospy
import tf

from math import pi, cos, sin
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from eod_pkg.msg import Wheels_Vel
from std_msgs.msg import Byte


class OdometryClass():

	def __init__(self):

		rospy.init_node("odometry")

		# ------ PARAMETRI ------
		# frequenza del loop 
		self.freq = rospy.get_param('eod/loop_freq/odom')
		self.rate = rospy.Rate(self.freq)

		# parametri fisici
		self.l = rospy.get_param('eod/physics/l') # distanza tra ruote
		self.r = rospy.get_param('eod/physics/r') # raggio ruota
		self.tick_encoder = rospy.get_param('eod/hs_interface/n_tick') # numero tick encoder
		self.step = (2.0*pi*self.r)/self.tick_encoder # passo encoder

		# ------ VARIABILI ------
		self.tick_dx = 0
		self.tick_sx = 0

		# Variabili odometria
		self.x = 0.0
		self.y = 0.0
		self.th = 0.0
		self.vx = 0.0
		self.vy = 0.0
		self.vth = 0.0

		# ------ TOPIC ------
		rospy.Subscriber('tick_dx', Byte, self.tick_dx_callback)
		rospy.Subscriber('tick_sx', Byte, self.tick_sx_callback)
		rospy.Subscriber('wheels_velocity', Wheels_Vel, self.wheels_vel_callback)

		self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
		self.odom_broadcaster = tf.TransformBroadcaster()


	def loop(self):
		while not rospy.is_shutdown():

			self.update_pose()

			# le velocita' vx,vy,vth vengono aggiornate nella callback
			# del subscriber al topic eod/wheels_vel

			self.publish_odom()

			self.rate.sleep()

	def update_pose(self):
		nr = self.tick_dx
		nl = self.tick_sx

		# azzeramento tick
		self.tick_dx = 0
		self.tick_sx = 0

		if nr == nl:
			# se il dual drive va indietro nr diventa negativo
			# quindi non c'e' bisogno di sottrarre pi radianti 
			# all'angolo theta.
			self.x = self.x + self.step*nr*cos(self.th)
			self.y = self.y + self.step*nr*sin(self.th)
			#self.th = self.th
		else:
			R = (self.l/2.0) * ((nr+nl)/(nr-nl)) # distanza da ICC al centro dell'asse delle ruote
			wdt = (self.step/self.l)*(nr-nl)   # theta' = theta + w*dt, omega:=w

			self.x = self.x - R*sin(self.th) + R*sin(self.th+wdt)
			self.y = self.y + R*cos(self.th) - R*cos(self.th+wdt)
			self.th = self.th + wdt


	def publish_odom(self):

		current_time = rospy.Time.now()

		# pubblica la posa a tf
		odom_trans = TransformStamped()
		odom_trans.header.stamp = current_time
		odom_trans.header.frame_id = 'odom'
		odom_trans.child_frame_id = 'base_footprint'

		odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
		odom_trans.transform.translation.x = self.x
		odom_trans.transform.translation.y = self.y
		odom_trans.transform.translation.z = 0.0
		odom_trans.transform.rotation = odom_quat

		self.odom_broadcaster.sendTransform((self.x, self.y, 0),tf.transformations.quaternion_from_euler(0, 0, self.th),current_time, 'base_footprint', 'odom')

		
		# pubblica il messaggio Odometry sul topic eod/odom
		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = 'base_footprint'
		odom.child_frame_id = 'odom'

		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0.0
		
		odom.pose.pose.orientation.x = odom_quat[0];
		odom.pose.pose.orientation.y = odom_quat[1];
		odom.pose.pose.orientation.z = odom_quat[2];
		odom.pose.pose.orientation.w = odom_quat[3];

		odom.twist.twist.linear.x = self.vx
		odom.twist.twist.linear.y = self.vy
		odom.twist.twist.angular.z = self.vth

		self.odom_pub.publish(odom)

	def tick_dx_callback(self, msg):
		self.tick_dx += msg.data

	def tick_sx_callback(self, msg):
		self.tick_sx += msg.data

	def wheels_vel_callback(self, msg):
		vr = msg.right
		vl = msg.left

		v = (vr+vl)/2

		self.vx = v * cos(self.th)
		self.vy = v * sin(self.th)
		self.vth = (vr-vl)/self.l


if __name__ == '__main__':
	o = OdometryClass()
	o.loop()


