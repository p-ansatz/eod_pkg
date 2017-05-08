#!/usr/bin/env python

import rospy
import actionlib
import points

from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from geometry_msgs.msg import Point

import eod_pkg.srv

class Waypoint(State):
	def __init__(self, position, orientation):
		
		State.__init__(self, outcomes=['success','abort'])
		
		# ------ SERVIZI ------
		rospy.wait_for_service('goto_point_srv')
		self.goto_point = rospy.ServiceProxy('goto_point_srv', BaseGoal)
				
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
		return self.goto_point(self.goal)
		
class GotoTarget(State):
	def __init__(self, position, orientation):
		
		State.__init__(self, outcomes=['success','abort'])
		
		# ------ SERVIZI ------
		rospy.wait_for_service('goto_point_srv')
		self.goto_point = rospy.ServiceProxy('goto_point_srv', BaseGoal)
		
	def execute(self, userdata):
		target_point = points.target_point

		# Define the goal
		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.pose.position.x = target_position[0]
		self.goal.target_pose.pose.position.y = target_position[1]
		self.goal.target_pose.pose.position.z = 0.0
		
		self.goal.target_pose.pose.orientation.x = target_orientation[0]
		self.goal.target_pose.pose.orientation.y = target_orientation[1]
		self.goal.target_pose.pose.orientation.z = target_orientation[2]
		self.goal.target_pose.pose.orientation.w = target_orientation[3]

		return self.goto_point(self.goal)

class GetObject(State):
	def __init__(self):
		
		State.__init__(self, outcomes=['success','abort'])
		
		# ------ SERVIZI ------
		rospy.wait_for_service('pick_object_srv')
		self.pick_object = rospy.ServiceProxy('pick_object_srv', ArmGoal)
		
	def execute(self, userdata):
		object_point = points.object_point

		# Define the goal
		# self.goal = ??
		return self.pick_object(self.goal)

if __name__ == '__main__':
	rospy.init_node('state_machine')
	SM = StateMachine(['success','abort'])
	
	points.init()
	waypoints = points.waypoints

	with SM:
		for i,w in enumerate(waypoints):
			StateMachine.add(w[0], Waypoint(w[1], w[2]), transitions={'success':waypoints[(i + 1)% len(waypoints)][0],'abort':'goto_target'})
		StateMachine.add('goto_target', GotoTarget() , transitions={'success':'get_object'})
		StateMachine.add('get_object', GetObject(), transitions={'success':waypoints[0][0]})

	SM.execute()