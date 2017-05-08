#!/usr/bin/env python


import rospy
from eod_pkg.msg import Wheels_Vel
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class RefTransformer():

	def __init__(self):
		rospy.init_node('ref_transformer')

		# ------ PARAMETRI ------
		# frequenza del loop
		self.freq = rospy.get_param('eod/loop_freq/default', 1)
		self.rate = rospy.Rate(self.freq)
		self.in_place_min_vel = 0.6 # minima velocita' di rotazione su se stesso
		self.theta_min_vel = 0.2 # minima velocita' di rotazione

		# parametri fisici
		self.l = rospy.get_param('eod/physics/l') # distanza tra ruote

		# ------ VARIABILI ------
		self.v_dx = 0.0
		self.v_sx = 0.0

		# ------ TOPIC ------
		rospy.Subscriber('cmd_vel', Twist, self.transform_callback)
		
		self.v_target_pub = rospy.Publisher('v_target', Wheels_Vel, queue_size=10)
		
	def loop(self):
		while not rospy.is_shutdown():
				
			self.rate.sleep()

	def transform_callback(self,msg):
		vx = msg.linear.x # velocita' frontale del dual drive
		vth = msg.angular.z

		if vx == 0:
		#gira su se stesso
			
			# la rotazione su se' stesso non puo' essere inferiore a in_place_min_vel
			if abs(vth) < self.in_place_min_vel:
				if vth > 0.0:
					vth += self.in_place_min_vel
				elif vth < 0.0:
					vth -= self.in_place_min_vel
			
			v_dx = vth * self.l/2.0 
			v_sx = (-1) * v_dx
		elif vth == 0:
			# procede diritto avanti o dietro
			v_dx = v_sx = vx
		else:
			# si muove su una curva
				
			if abs(vth) < self.theta_min_vel:
				if vth > 0.0:
					vth += self.theta_min_vel
				elif vth < 0.0:
					vth -= self.theta_min_vel

			v_dx = vx + vth * self.l/2.0
			v_sx = vx - vth * self.l/2.0

		# si aggiorna il riferimento solo se e' diverso dal precedente
		if not ((v_dx == self.v_dx) and (v_sx == self.v_sx)):
			self.v_dx = v_dx
			self.v_sx = v_sx
		
			msg_vel = Wheels_Vel()
			msg_vel.right = self.v_dx
			msg_vel.left = self.v_sx
			self.v_target_pub.publish(msg_vel)

if __name__ == '__main__':
	rt = RefTransformer()
	rt.loop()

