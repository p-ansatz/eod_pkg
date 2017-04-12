#!/usr/bin/env python

import rospy
import numpy

class MotorController():
	
	def __init__(self):
		rospy.init_node('eod/motor_controller')

		# ------ PARAMETRI ------
		# frequenza del loop 
		self.freq = rospy.get_param('eod/loop_freq/motor_ctrl', 0.1)
		self.rate = rospy.Rate(self.freq)

		# parametri pid dx
		self.kp_dx = rospy.get_param('eod/pid_dx/Kp') 
		self.ki_dx = rospy.get_param('eod/pid_dx/Ki') 
		self.kd_dx = rospy.get_param('eod/pid_dx/Kd')
		self.vMin_dx = rospy.get_param('eod/pid_dx/vMin') 
		self.vMax_dx = rospy.get_param('eod/pid_dx/vMax') 

		# parametri pid sx
		self.kp_sx = rospy.get_param('eod/pid_sx/Kp') 
		self.ki_sx = rospy.get_param('eod/pid_sx/Ki') 
		self.kd_sx = rospy.get_param('eod/pid_sx/Kd')
		self.vMin_sx = rospy.get_param('eod/pid_sx/vMin') 
		self.vMax_sx = rospy.get_param('eod/pid_sx/vMax') 

		# ------ VARIABILI ------
		self.old_time = rospy.Time.now()
		self.pid_dx = PID(kp_dx, ki_dx, kd_dx, vMin_dx, vMax_dx)
		self.pid_sx = PID(kp_sx, ki_sx, kd_sx, vMin_sx, vMax_sx)
		self.v_dx = 0.0
		self.v_sx = 0.0
		self.ref_dx = 0.0
		self.ref_sx = 0.0

		# ------ TOPIC ------
		rospy.Subscriber('eod/v_target', Wheels_Vel, self.v_target_callback)
		rospy.Subscriber('eod/wheels_velocity', Wheels_Vel, self.wheels_velocity_callback)
		
		# il messaggio Wheels_Vel contiene due valori di tensione
		self.cmd_pub = rospy.Publisher('eod/motor_cmd', Wheels_Vel, queue_size=10)

	def loop(self):

		while not rospy.is_shutdown():

			# calcolo dt e aggiornamento 
			dt = calculate_dt()

			# calcolo della legge di controlo
			v_input_dx = self.pid_dx.calculate( v_dx, ref_dx, dt)
			v_input_sx = self.pid_sx.calculate( v_sx, ref_sx, dt)

			# controllo tra segno della tensione e direzione di marcia
			v_input_dx = saturation(v_input_dx, numpy.sign(ref_dx))
			v_input_sx = saturation(v_input_sx, numpy.sign(ref_sx))

			# normalizzazione input tra [0,1]
			v_input_dx = v_input_dx/self.vMax_dx
			v_input_sx = v_input_sx/self.vMax_sx

			# pubblicazione dei segnali di controllo
			publish_motor_cmd(v_input_dx, v_input_sx)

			self.rate.sleep()
			
	def calculate_dt(self):

		current_time = rospy.Time.now()
		dt = (current_time - self.old_time).toSec()
		self.old_time = current_time
		return dt

	def saturation(self, v_input, sign):
		# se l'input da fornire al motore e' opposto alla direzione di marcia
		# si preferisce fornire una tensione nulla
		if ((sign > 0) and (v_input < 0)) or ((sign < 0) and (v_input > 0)):
			v_input = 0.0
		
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

