#!/usr/bin/env python
import rospy
import actionlib
import tf

from collections import deque
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class GoalGovernor():
	def __init__(self):
		rospy.init_node('goal_governor')

		# ------ PARAMETRI ------
		# frequenza del loop (
		self.freq = rospy.get_param('eod/loop_freq/default', 10)
		self.rate = rospy.Rate(self.freq)

		# ------ VARIABILI ------
		self.sent_place_goal = False
		self.good_place_goal = False
		self.pick_phase_allowed = True
		self.good_arm_goal = False 
		self.operation_completed = False
		self.eps = 0.01

		# inizializzazione fittizia della coda con 5 elementi
		self.fifo_list = deque( [ [1.0,2.0,3.0,4.0,5.0,6.0,7.0], [21.0,22.0,23.0,24.0,25.0,26.0,27.0],
		[31.0,32.0,33.0,34.0,35.0,36.0,37.0],[41.0,42.0,43.0,44.0,45.0,46.0,47.0],[51.0,52.0,53.0,54.0,55.0,56.0,57.0] ])

		# ------ TOPIC ------
		rospy.Subscriber('objects_stamped_poses', PoseStamped, self.callback_send_goal)
		rospy.Subscriber('posa_raggiunta', Bool, self.callback_reached_pose)

		self.arm_pub = rospy.Publisher('arm_goal_pose', PoseStamped, queue_size=10)

		# ------ ACTIONS ------
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.client.wait_for_server()

		# ------ TF ------
		self.tl = tf.TransformListener()

	def loop(self):
		while not rospy.is_shutdown():

			self.place_phase()

			self.pick_phase()

			if self.operation_completed:
				rospy.signal_shutdown("work done")

			self.rate.sleep()

	def callback_send_goal(self, msg):
			self.msg = msg
			x = msg.position.x
			y = msg.position.y
			z = msg.position.z
			rx = msg.orientation.x
			ry = msg.orientation.y
			rz = msg.orientation.z
			rw = msg.orientation.w

			self.fifo_list.popleft()	
			self.fifo_list.append([x,y,z,rx,ry,rz,rw])
	
	def callback_reached_pose(self, msg):		
		self.operation_completed = msg.data

	def place_phase(self):
		if !self.sent_place_goal:

			place_goal = check_place()
			
			if self.good_place_goal:
				goal_pose = MoveBaseGoal()
				goal_pose.target_pose.header.frame_id = 'map'
				goal_pose.target_pose.pose.position.x = place_goal[0]
				goal_pose.target_pose.pose.position.y = place_goal[1]
				goal_pose.target_pose.pose.position.z = place_goal[2]
				goal_pose.target_pose.pose.orientation.x = place_goal[3]
				goal_pose.target_pose.pose.orientation.y = place_goal[4]
				goal_pose.target_pose.pose.orientation.z = place_goal[5]
				goal_pose.target_pose.pose.orientation.w = place_goal[6]
				
				self.client.send_goal(goal_pose)
				self.sent_place_goal=True
				self.client.wait_for_result()
				result = client.get_state()
				if result == 3: # SUCCEEDED
					 self.pick_phase_allowed = True

	def pick_phase(self):
		if self.pick_phase_allowed:

			arm_goal = check_pick()
		
			if self.good_arm_goal:
				arm_goal_pose = PoseStamped()
				arm_goal.header.frame_id = 'map'
				arm_goal.pose.position.x = arm_goal[0]
				arm_goal.pose.position.y = arm_goal[1]
				arm_goal.pose.position.z = arm_goal[2]
				arm_goal.pose.orientation.x = arm_goal[3]
				arm_goal.pose.orientation.y = arm_goal[4]
				arm_goal.pose.orientation.z = arm_goal[5]
				arm_goal.pose.orientation.w = arm_goal[6]

				pose_in_arm_base = self.tl.transformPose("arm_base", arm_goal_pose)
				self.arm_pub.publish(pose_in_arm_base)
				self.pick_phase_allowed = False

	
	def check_place(self):

		# numero di pose nella lista
		n_pose = len(self.fifo_list) 

		found = True

		# controllo solo i primi tre valori x,y,z
		for i in range(0, 3): # i€[0, 2] 
			# per ogni coordinata controllo che nessuna coppia di
			# valori differisca più della soglia eps
			for j in range(0, n_pose-1): # j€[0, n_pose-2]
				for k in range(j+1, n_pose): # k€[0, n_pose-1]
					
					if abs( self.fifo_list[j][i]-self.fifo_list[k][i] ) >= self.eps:
							found = False

		self.good_place_goal = found
		return self.fifo_list[n_pose-1] # ritorniamo l'ultima posa ricevuta

	
	def check_pick(self):
		
		# numero di pose nella lista
		n_pose = len(self.fifo_list) 
		
		found = True

		# controllo solo i primi tre valori x,y,z
		for i in range(0, 3): # i€[0, 2] 
			# per ogni coordinata controllo che nessuna coppia di
			# valori differisca più della soglia eps
			for j in range(0, n_pose-1):  # j€[0, n_pose-2]
				for k in range(j+1, n_pose): # k€[0, n_pose-1]
					
					if abs( self.fifo_list[j][i]-self.fifo_list[k][i] ) >= self.eps:
							found = False

		self.good_arm_goal = found
		return self.fifo_list[n_pose-1] # ritorniamo l'ultima posa ricevuta
		

if __name__ == '__main__':
	gg = GoalGovernor()
	gg.loop()