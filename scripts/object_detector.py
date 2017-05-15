#!/usr/bin/env python
import rospy
import actionlib
import tf
import global_area

from collections import deque
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

class ObjectDetector():
	def __init__(self):
		rospy.init_node('object_detector', log_level=rospy.INFO)

		# ------ PARAMETRI ------
		# frequenza del loop ,		
		self.freq = rospy.get_param('eod/loop_freq/default', 10)
		self.rate = rospy.Rate(self.freq)

		# ------ VARIABILI ------
		self.good_place_goal = False
		self.eps = 0.01

		# inizializzazione fittizia della coda con 5 elementi
		self.fifo_list = deque( [ [1.0,2.0,3.0,4.0,5.0,6.0,7.0], [21.0,22.0,23.0,24.0,25.0,26.0,27.0],
		[31.0,32.0,33.0,34.0,35.0,36.0,37.0],[41.0,42.0,43.0,44.0,45.0,46.0,47.0],[51.0,52.0,53.0,54.0,55.0,56.0,57.0] ])

		# ------ TOPIC ------
		rospy.Subscriber('objects_stamped_poses', PoseStamped, self.object_callback)
		
	def loop(self):
		while not rospy.is_shutdown():

			self.rate.sleep()

	def object_callback(self, msg):
			self.msg = msg
			x = msg.pose.position.x
			y = msg.pose.position.y
			z = msg.pose.position.z
			rx = msg.pose.orientation.x
			ry = msg.pose.orientation.y
			rz = msg.pose.orientation.z
			rw = msg.pose.orientation.w

			self.fifo_list.popleft()	
			self.fifo_list.append([x,y,z,rx,ry,rz,rw])

			place_goal = self.check_object()
			
			if self.good_place_goal and (not global_area.target_flag):
				self.good_place_goal = False
				global_area.target_flag = True
				out_file = open('target.txt','w')
				out_file.write(" ".join(str(x) for x in place_goal))
				out_file.close()
				global_area.modify_target_point(place_goal)
				global_area.abort_move()


	
	def check_object(self):

		# numero di pose nella lista
		n_pose = len(self.fifo_list) 

		found = True

		# controllo solo i primi tre valori x,y,z
		for i in range(0, 3): # i in [0, 2] 
			# per ogni coordinata controllo che nessuna coppia di
			# valori differisca piu' della soglia eps
			for j in range(0, n_pose-1): # j in [0, n_pose-2]
				for k in range(j+1, n_pose): # k in [0, n_pose-1]
					
					if abs( self.fifo_list[j][i]-self.fifo_list[k][i] ) >= self.eps:
							found = False

		self.good_place_goal = found
		return self.fifo_list[n_pose-1] # ritorniamo l'ultima posa ricevuta

	
if __name__ == '__main__':
	od = ObjectDetector()
	od.loop()