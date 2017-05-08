#!/usr/bin/env python

import rospy
import points
import eod_pkg.srv

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from std_msgs.msg import String

class StateMachineServer():

	def __init__(self):
		rospy.init_node('sms')


		# ------ PARAMETRI -----
		self.results = {3 : 'success',4 : 'abort'}

		# ------ TOPIC ------
		# Topic sul quale è presente l'informazione utilizzata per abortire la fase di pattugliamento
		rospy.Subscriber('NAME', TYPE, self.topic_callback)

		# ------ SERVIZI ------	
		goto_point_srv = rospy.Service('goto_point_srv', BaseGoal, self.goto_callback)
		pick_object_srv = rospy.Service('pick_object_srv', ArmGoal, self.pick_callback)

		# ------ AZIONI ------
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.client.wait_for_server()

	def loop(self):
		rospy.spin()

	def goto_callback(self, msg):
		# msg contiene un MoveBaseGoal
		self.client.send_goal(msg.goal)
		self.client.wait_for_result()

		return self.results[self.client.get_state()]


	def pick_callback(self, msg):
		# msg contiene un Point

if __name__ == "__main__":
	sms = StateMachineServer()
	sms.loop()