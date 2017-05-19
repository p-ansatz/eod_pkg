#!/usr/bin/env python
import rospy
import tf
import math

from geometry_msgs.msg import PoseStamped

class GoalPosePublisher():
	def __init__(self):
		rospy.init_node('goal_pose_publisher')

		# ------ PARAMETRI ------
		self.rate = rospy.Rate(5.0)

		# ------ VARIABILI ------
		self.object_pose = PoseStamped()
		self.object_pose_received = False
		
		# ------ TOPIC ------
		rospy.Subscriber('objects_stamped_poses', PoseStamped, self.callback_object_detected)
		self.goal_pub = rospy.Publisher('goal_pose', PoseStamped, queue_size=10)

		# ------ TF ------
		self.tl = tf.TransformListener()
		self.br = tf.TransformBroadcaster()

	def loop(self):
		while not rospy.is_shutdown():
			
			if self.object_pose_received:

				self.create_goal_frame()
				
				self.publish_goal_pose()
				
				self.rate.sleep()

	def create_goal_frame(self):
		
		position = (self.object_pose.pose.position.x, \
			self.object_pose.pose.position.y, self.object_pose.pose.position.z)
		orientation = (self.object_pose.pose.orientation.x, self.object_pose.pose.orientation.y,\
			self.object_pose.pose.orientation.z, self.object_pose.pose.orientation.w)
		self.br.sendTransform(position, orientation, self.object_pose.header.stamp,\
		 "fake_frame", "map")
		self.br.sendTransform((0.3, 0.0, 0.0),
                 (0.0, 0.0, 0.0, 1.0),
                 rospy.Time.now(),
                 "goal_frame",
                 "fake_frame")

	def publish_goal_pose(self):
		try:
			now = rospy.Time.now() - rospy.Duration(5.0)
			self.tl.waitForTransform("/map", "/goal_frame", now, rospy.Duration(1.0))
			(trans, rot) = self.tl.lookupTransform("/map", "/goal_frame", now)
			
			# Il sistema di riferimento goal_frame viene proiettato sul piano.
			
			# le rotazioni sulla asse x ed y vengono annullate 
			quaternion = (rot[0], rot[1], rot[2], rot[3])
			euler = tf.transformations.euler_from_quaternion(quaternion)
			roll = 0.0 # asse x
			pitch = 0.0 # asse y
			# il robot deve andare nella direzione opposta a quella dell'asse X
			yaw = euler[2] + math.pi 

			quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

			goal_pose = PoseStamped()
			goal_pose.header.stamp = rospy.Time.now()
			goal_pose.header.frame_id = "map"
			goal_pose.pose.position.x = trans[0]
			goal_pose.pose.position.y = trans[1]
			goal_pose.pose.position.z = 0.0 # il goal giace sul piano 
			goal_pose.pose.orientation.x = quaternion[0]
			goal_pose.pose.orientation.y = quaternion[1]
			goal_pose.pose.orientation.z = quaternion[2]
			goal_pose.pose.orientation.w = quaternion[3]

			#print "goal_pose"
			#print goal_pose.pose.orientation

			self.goal_pub.publish(goal_pose)
			
		except (tf.Exception, tf.LookupException, tf.ConnectivityException):
			pass



	def callback_object_detected(self, msg):

		self.object_pose.header.stamp       = msg.header.stamp              
		self.object_pose.header.frame_id    = msg.header.frame_id        
		self.object_pose.pose.position.x    = msg.pose.position.x
		self.object_pose.pose.position.y    = msg.pose.position.y
		self.object_pose.pose.position.z    = msg.pose.position.z
		self.object_pose.pose.orientation.x = msg.pose.orientation.x
		self.object_pose.pose.orientation.y = msg.pose.orientation.y
		self.object_pose.pose.orientation.z = msg.pose.orientation.z
		self.object_pose.pose.orientation.w = msg.pose.orientation.w

		#print "object_pose"
		#print self.object_pose.pose.orientation

		self.object_pose_received = True 

if __name__ == '__main__':
	gpp = GoalPosePublisher()
	gpp.loop()
