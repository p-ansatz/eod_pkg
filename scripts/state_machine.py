#!/usr/bin/env python

import rospy
import actionlib
import global_area
import tf

from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Int16
from geometry_msgs.msg import Point, PoseStamped
from eod_pkg.msg import MoveArmAction, MoveArmGoal, MoveArmResult

class Waypoint(State):
	
	def __init__(self, position, orientation):
		
		State.__init__(self, outcomes=['success','abort'])
		
		rospy.init_node('waypoint')
		
		global_area.init()
		
		# ------ PARAMETRI -----
		self.results = {3 : 'success',4 : 'abort'}

		# Define the goal
		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.pose.position.x = position[0]
		self.goal.target_pose.pose.position.y = position[1]
		self.goal.target_pose.pose.position.z = 0.0
		
		self.goal.target_pose.pose.orientation.x = orientation[0]
		self.goal.target_pose.pose.orientation.y = orientation[1]
		self.goal.target_pose.pose.orientation.z = orientation[2]
		self.goal.target_pose.pose.orientation.w = orientation[3]
	
	def execute(self, userdata):
		global_area.move_client.send_goal(self.goal)
		global_area.move_client.wait_for_result()

		return self.results[global_area.move_client.get_state()]

class GotoTarget(State):
	def __init__(self, position, orientation):
		
		State.__init__(self, outcomes=['success','abort'])
		
		global_area.init()
		
	def execute(self, userdata):
		target_point = global_area.get_target_point()

		# Define the goal
		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.pose.position.x = target_point[0][0]
		self.goal.target_pose.pose.position.y = target_point[0][1]
		self.goal.target_pose.pose.position.z = 0.0
		
		self.goal.target_pose.pose.orientation.x = target_point[1][0]
		self.goal.target_pose.pose.orientation.y = target_point[1][1]
		self.goal.target_pose.pose.orientation.z = target_point[1][2]
		self.goal.target_pose.pose.orientation.w = target_point[1][3]

		global_area.move_client.send_goal(self.goal)
		global_area.move_client.wait_for_result()

		return self.results[global_area.move_client.get_state()]

class GetObject(State):
	def __init__(self):
		
		State.__init__(self, outcomes=['success','abort'])

		# ------ PARAMETRI -----
		self.results = {3 : 'success',4 : 'abort'}

		# ------ AZIONI ------
		self.move_arm_client = actionlib.SimpleActionClient('move_arm', MoveArmAction)
		self.goal = MoveArmGoal()
		
		# ------ TF ------
		self.tl = tf.TransformListener()

	def execute(self, userdata):
		self.move_arm_client.wait_for_server()
		self.goal.p = self.create_goal()
		self.goal.cmd = 1
		self.move_arm_client.send_goal(self.goal)
		self.move_arm_client.wait_for_result()

		return self.results[self.move_arm_client.get_result()]
		
	def create_goal(self):
		point = global_area.get_target_point()
		arm_goal_pose = PoseStamped()
		arm_goal.header.frame_id = 'map'
		arm_goal.pose.position.x = point[0]
		arm_goal.pose.position.y = point[1]
		arm_goal.pose.position.z = point[2]
		arm_goal.pose.orientation.x = point[3]
		arm_goal.pose.orientation.y = point[4]
		arm_goal.pose.orientation.z = point[5]
		arm_goal.pose.orientation.w = point[6]

		return self.tl.transformPose("arm_base", arm_goal_pose)

if __name__ == '__main__':
	rospy.init_node('state_machine')
	SM = StateMachine(['success','abort'])
	
	global_area.init()
	global_area.move_client.wait_for_server()
	waypoints = global_area.waypoints

	with SM:
		for i,w in enumerate(waypoints):
			StateMachine.add(w[0], Waypoint(w[1], w[2]), transitions={'success':waypoints[(i + 1)% len(waypoints)][0],'abort':'goto_target'})
		StateMachine.add('goto_target', GotoTarget() , transitions={'success':'get_object'})
		StateMachine.add('get_object', GetObject(), transitions={'success':waypoints[0][0]})

	SM.execute()