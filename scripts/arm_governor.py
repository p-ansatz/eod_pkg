#! /usr/bin/env python

import rospy
import actionlib
from eod_pkg.msg import MoveArmGoal, MoveArmAction, MoveArmResult
from eod_pkg.msg import ArmPoseGoal, ArmPoseAction, ArmPoseResult
from eod_pkg.msg import ArmClawGoal, ArmClawAction, ArmClawResult

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
		self.pose_client  = actionlib.SimpleActionClient('arm_pos', ArmPoseAction)
		self.pose_client.wait_for_server()
		self.claw_client  = actionlib.SimpleActionClient('arm_claw', ArmClawAction)
		self.claw_client.wait_for_server()

	def move_arm_callback(self, msg):
		if (msg.cmd == 1):
			# Chiudi pinza
			goal = ArmClawGoal()
			goal.open = False
			self.claw_client.send_goal(goal)
			self.claw_client.wait_for_result()
			result = MoveArmResult()
			result.res = 'success'
			self.move_arm_server.set_succeeded(result)
		elif (msg.cmd == 2):
			# Apri pinza
			goal = ArmClawGoal()
			goal.open = True
			self.claw_client.send_goal(goal)
			self.claw_client.wait_for_result()
			result = MoveArmResult()
			result.res = 'success'
			self.move_arm_server.set_succeeded(result)
		elif (msg.cmd == 3)
			# Posiziona braccio
			goal = msg.p 
			self.pose_client.send_goal(goal)
			self.pose_client.wait_for_result()
			result = MoveArmResult()
			result.res = 'success'
			self.move_arm_server.set_succeeded(result)
		elif (msg.cmd == 4)
			# Prendi oggetto
			# apri pinza
			goal = ArmClawGoal()
			goal.open = True
			self.claw_client.send_goal(goal)
			self.claw_client.wait_for_result()
			# posiziona braccio
			goal = msg.p 
			self.pose_client.send_goal(goal)
			self.pose_client.wait_for_result()
			# chiudi pinza
			goal = ArmClawGoal()
			goal.open = False
			self.claw_client.send_goal(goal)
			self.claw_client.wait_for_result()
			result = MoveArmResult()
			result.res = 'success'
			self.move_arm_server.set_succeeded(result)


    def loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
	ag = ArmGovernor()
	ag.loop()
