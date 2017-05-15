#!/usr/bin/env python

import rospy
import numpy
from eod_pkg.msg import Wheels_Vel
from pid import PID

class MotorController():
	
	def __init__(self):
		rospy.init_node('motor_controller',log_level=rospy.INFO)

		# ------ PARAMETRI ------
		# frequenza del loop 
		self.freq = rospy.get_param('/eod/loop_freq/motor_ctrl')
		self.rate = rospy.Rate(self.freq)

		# parametri pid dx
		self.kp_dx = rospy.get_param('/eod/pid_dx/Kp') 
		self.ki_dx = rospy.get_param('/eod/pid_dx/Ki') 
		self.kd_dx = rospy.get_param('/eod/pid_dx/Kd')
		self.vMin_dx = rospy.get_param('/eod/pid_dx/v_min') 
		self.vMax_dx = rospy.get_param('/eod/pid_dx/v_max') 

		# parametri pid sx
		self.kp_sx = rospy.get_param('/eod/pid_sx/Kp') 
		self.ki_sx = rospy.get_param('/eod/pid_sx/Ki') 
		self.kd_sx = rospy.get_param('/eod/pid_sx/Kd')
		self.vMin_sx = rospy.get_param('/eod/pid_sx/v_min') 
		self.vMax_sx = rospy.get_param('/eod/pid_sx/v_max') 

		# ------ VARIABILI ------
		self.old_time = rospy.Time.now()
		self.pid_dx = PID(self.kp_dx, self.ki_dx, self.kd_dx, self.vMin_dx, self.vMax_dx)
		self.pid_sx = PID(self.kp_sx, self.ki_sx, self.kd_sx, self.vMin_sx, self.vMax_sx)
		self.v_dx = 0.0
		self.v_sx = 0.0
		self.ref_dx = 0.0
		self.ref_sx = 0.0

		# ------ TOPIC ------
		rospy.Subscriber('v_target', Wheels_Vel, self.v_target_callback)
		rospy.Subscriber('wheels_velocity', Wheels_Vel, self.wheels_velocity_callback)
		
		# il messaggio Wheels_Vel contiene due valori di tensione
		self.cmd_pub = rospy.Publisher('motor_cmd', Wheels_Vel, queue_size=10)

	def loop(self):

		while not rospy.is_shutdown():

			# calcolo dt e aggiornamento 
			dt = self.calculate_dt()

			# calcolo della legge di controlo
			v_input_dx = self.pid_dx.calculate(self.v_dx, self.ref_dx, dt)
			v_input_sx = self.pid_sx.calculate(self.v_sx, self.ref_sx, dt)

			# controllo tra segno della tensione e direzione di marcia
			v_input_dx = self.saturation(v_input_dx, numpy.sign(self.ref_dx))
			v_input_sx = self.saturation(v_input_sx, numpy.sign(self.ref_sx))

			# normalizzazione input tra [0,1]
			v_input_dx = v_input_dx/self.vMax_dx
			v_input_sx = v_input_sx/self.vMax_sx

			# pubblicazione dei segnali di controllo
			self.publish_motor_cmd(v_input_dx, v_input_sx)

			self.rate.sleep()
			
	def calculate_dt(self):

		current_time = rospy.Time.now()
		dt = (current_time - self.old_time).to_sec()
		self.old_time = current_time
		return dt

	def saturation(self, v_input, sign):
		# se l'input da fornire al motore e' opposto alla direzione di marcia
		# si preferisce fornire una tensione nulla
		if ((sign > 0) and (v_input < 0)) or ((sign < 0) and (v_input > 0)):
			return 0.0
		else:
			return v_input
        
	def publish_motor_cmd(self, v_input_dx, v_input_sx):
		# anche se il messaggio e' di tipo Wheels_Vel stiamo
		# pubblicando il valore di due tensioni
		msg_cmd = Wheels_Vel()
		msg_cmd.right = v_input_dx
		msg_cmd.left = v_input_sx
		self.cmd_pub.publish(msg_cmd)

	def v_target_callback(self, msg):
		self.ref_dx = msg.right
		self.ref_sx = msg.left

	def wheels_velocity_callback(self, msg):
		self.v_dx = msg.right
		self.v_sx = msg.left


if __name__ == '__main__':
	mc = MotorController()
	mc.loop()

