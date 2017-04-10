#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32

class Security_Stop():

	def __init__(self):
		rospy.init_node('security_stop')

		# Tempo massimo riferimento
		self.time_max = rospy.get_param('~time_max',5.0)
		
		# Tempo ultima pubblicazione riferimento
		self.last_time = rospy.Time.now().to_sec()

		# Topic
		rospy.Subscriber('vel_ref', Float32, self.callback)
		self.pub = rospy.Publisher('vel_ref', Float32, queue_size=10)


	def spin(self):
		while not rospy.is_shutdown():
			rospy.sleep(self.time_max)
			
			if abs(rospy.Time.now().to_sec() - self.last_time) > self.time_max*0.8:
				self.pub.publish(0.0)

	def callback(self,msg):
		self.last_time = rospy.Time.now().to_sec()

if __name__ == '__main__':
	s_stop = Security_Stop()
	s_stop.spin()