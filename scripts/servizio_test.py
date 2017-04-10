#!/usr/bin/env python

import rospy
from eod_pkg.srv import VelRuota,VelRuotaResponse,VelRuotaRequest

class Srv_Test():

	def __init__(self):
		rospy.init_node('servizio_test')
			
		self.vel = 0.0
		self.t_old = rospy.Time.now()

		self.fc = rospy.get_param('~fc',30)
		self.rc = rospy.Rate(self.fc)

		rospy.wait_for_service('vel_encoder')
		self.encoder = rospy.ServiceProxy('vel_encoder',VelRuota)

	def spin(self):
		while not rospy.is_shutdown():
			self.dt = (rospy.Time.now() - self.t_old).to_sec()
			self.t_old = rospy.Time.now()
			self.vel = self.encoder(self.dt).vel
			print("%1.15f" % self.vel)			
			self.rc.sleep()

if __name__ == '__main__':
	st = Srv_Test()
	st.spin()