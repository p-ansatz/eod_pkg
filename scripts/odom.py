#!/usr/bin/env python

import rospy
import math
from eod_pkg.msg import Ticks
from eod_pkg.msg import Posa # Sostituire con nav_msgs

class Odometria():

	def __init__(self):

		rospy.init_node("odometria")

		# Variabili Posa
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0

		# Parametri Fisici
		self.l = rospy.get_param('~l',0.1) # Distanza tra ruote
		self.r = rospy.get_param('~r',0.1) # Raggio ruota
		self.N = rospy.get_param('~N',20) # Numero tick encoder
		self.step = (2.0*pi*self.r)/self.N # Passo encoder

		# Topic
		rospy.Subscriber('tick_ruote', Ticks, self.tick_callback)
		self.pub_posa = rospy.Publisher('posa', Posa, queue_size=10)

	def spin(self):
		rospy.spin()

	def tick_callback(self, tick):
		nr = tick.dx
		nl = tick.sx

		if nr == nl:
			self.x = self.x - self.step*cos(self.theta)
			self.y = self.y + self.step*sin(self.theta)
			#self.theta = self.theta
		else:
			k1 = ((self.l/2.0)*(nr+nl))/(nr-nl)
			k2 = (self.step/self.l)*(nr-nl)

			self.x = self.x - k1*sin(self.theta) + k1*sin(self.theta+k2)
			self.y = self.y + k1*cos(self.theta) - k1*cos(self.theta+k2)
			self.theta = self.theta + k2

		posa = Posa()
		posa.x = self.x
		posa.y = self.y
		posa.theta = self.theta

		pub_posa.publish(posa)


if __name__=='__main__':
	odom = Odometria()
	odom.spin()