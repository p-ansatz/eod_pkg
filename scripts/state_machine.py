#!/usr/bin/env python

import rospy
import actionlib
import global_area
import tf
import time
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Int16
from geometry_msgs.msg import Point, PoseStamped
from arm_pkg.msg import *
from math import pi

class Waypoint(State):
	
	def __init__(self, position, orientation):
		
		State.__init__(self, outcomes=['success','abort'])
		
		global_area.init()

		self.position=position
		
		# ------ PARAMETRI -----
		self.results = {3 : 'success', 2 : 'abort'}

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

		rospy.loginfo('vado in x:%f y:%f '%(self.position[0],self.position[1]) )

		global_area.target_flag = False

		global_area.move_client.send_goal(self.goal)
		time.sleep(1)
		global_area.move_client.wait_for_result()
				
		return self.results[global_area.move_client.get_state()]

class GotoTarget(State): 
	def __init__(self):
		
		State.__init__(self, outcomes=['success','abort'])
		
		# ------ PARAMETRI -----
		self.results = {3 : 'success', 4 : 'abort'}
		
	def execute(self, userdata):
		rospy.loginfo('goto target' )

		target_point = self.extract_target_point()

		# Define the goal
		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.pose.position.x = target_point[0]
		self.goal.target_pose.pose.position.y = target_point[1]
		self.goal.target_pose.pose.position.z = 0.0
		
		# Rotazione sistema di riferimento del target 
		# quaternion = (target_point[3], target_point[4],target_point[5],target_point[6])
		# euler = tf.transformations.euler_from_quaternion(quaternion)
		# # il target e' un punto del piano, il sistema di riferimento associato
		# # non e' ruotato rispetto a x ed y ma solo rispetto z
		# roll = 0.0 # asse x
		# pitch = 0.0 # asse y
		# yaw = euler[2] + pi # asse z 
		# quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		########
		#quaternion = (0.0,0.0, 0.70710678 , 0.70710678)
		self.goal.target_pose.pose.orientation.x = target_point[3]
		self.goal.target_pose.pose.orientation.y = target_point[4]
		self.goal.target_pose.pose.orientation.z = target_point[5]
		self.goal.target_pose.pose.orientation.w = target_point[6]
		
		print self.goal

		move_client1  = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		move_client1.wait_for_server()
		
		rospy.loginfo('vado in x:%f y:%f '%(target_point[0],target_point[1]) )
		move_client1.send_goal(self.goal)

		move_client1.wait_for_result()
		print('state: %d'%move_client1.get_state())
		print('status: %s'%move_client1.get_goal_status_text())
		return self.results[move_client1.get_state()]

	def extract_target_point(self):
		# Get target point - read from file
		lines = [line.rstrip('\n') for line in open('poses.txt')]
		list_str = lines[0].split(" ") #lines[0] contiene la posa da raggiungere
		#in_file.close()
		return map(float, list_str)


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

		# Creazione Goal per il braccio
		self.goal.p = self.create_goal()
		# cmd=1: chiudi pinza
		# cmd=2: apri pinza
		# cmd=3: posiziona braccio
		# cmd=4: prendi oggetto
		self.goal.cmd = 4

		self.move_arm_client.send_goal(self.goal)
		self.move_arm_client.wait_for_result()

		return self.results[self.move_arm_client.get_result()]
		
	def create_goal(self):

		object_point = self.extract_object_point()

		arm_goal = PoseStamped()
		arm_goal.header.frame_id = 'map'
		arm_goal.pose.position.x = object_point[0]
		arm_goal.pose.position.y = object_point[1]
		arm_goal.pose.position.z = object_point[2]
		# Non verranno utilizzate dal braccio
		arm_goal.pose.orientation.x = object_point[3]
		arm_goal.pose.orientation.y = object_point[4]
		arm_goal.pose.orientation.z = object_point[5]
		arm_goal.pose.orientation.w = object_point[6]

		# Trasformazione del frame dell'oggetto da 'map' a 'base_arm'
		pose_in_arm = self.tl.transformPose('/base_arm', arm_goal)

		point = Point()
		point.x = pose_in_arm.pose.position.x
		point.y = pose_in_arm.pose.position.y
		point.z = 0.05#pose_in_arm.pose.position.z

		return point

	def extract_object_point(self):
		# Get object point - read from file
		lines = [line.rstrip('\n') for line in open('poses.txt')]
		list_str = lines[1].split(" ") #lines[1] contiene la posa dall'oggetto
		#in_file.close()

		return map(float, list_str)


if __name__ == '__main__':
	rospy.init_node('state_machine')
	SM = StateMachine(['success','abort'])
	
	global_area.init()
	global_area.move_client.wait_for_server()
	waypoints = global_area.waypoints

	with SM:
		for i,w in enumerate(waypoints):
			StateMachine.add(w[0], Waypoint(w[1], w[2]), transitions={'success': waypoints[(i + 1)%len(waypoints)][0] ,'abort':'goto_target'})
		StateMachine.add('goto_target', GotoTarget() , transitions={'success':'get_object','abort':'goto_target'})
		StateMachine.add('get_object', GetObject(), transitions={'success':'one','abort':'get_object'})

	SM.execute()