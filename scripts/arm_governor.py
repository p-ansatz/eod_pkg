#! /usr/bin/env python

import rospy
import actionlib
from eod_pkg.msg import MoveArmGoal, MoveArmAction, MoveArmResult

from std_msgs.msg import String, Int16
from geometry_msgs.msg import Point

class ArmGovernor():
	def __init__(self):
		rospy.init_node('arm_governor')	

		# ------ PARAMETRI ------
    	# frequenza del loop 
        self.fq = rospy.get_param('/eod/loop_freq/default')
        self.rate = rospy.Rate(self.freq)

		# ----- AZIONI ------
		self.move_arm_server = actionlib.SimpleActionServer('move_arm', MoveArmAction, move_arm_callback, False)
		self.move_arm_server.start()

		# ------ TOPIC ------
		self.pose_pub = rospy.Publisher('prendi_oggetti', Point, queue_size=10)
		self.pinza_pub = rospy.Publisher('posiziona_pinza', Point, queue_size=10)
		rospy.Subscriber('operazione_terminata', Int16, self.op_term_callback)

	def move_arm_callback(self, goal):
		self.cmd = goal.cmd
		if (self.cmd == 1):
			# Prendi Oggetto
			self.pose_pub.publish(goal.p)
		elif (self.cmd == 2):
			# Posiziona Pinza
			self.pinza_pub.publish(goal.p)

	def op_term_callback(self, msg):
		if (self.cmd == msg):
			# Azione Terminata
			result = MoveArmResult()
			result.res = 'success'
			self.move_arm_server.set_succeeded(result)

    def loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
	ag = ArmGovernor()
	ag.loop()
