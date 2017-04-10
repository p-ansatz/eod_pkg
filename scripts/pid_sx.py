#!/usr/bin/env python


import rospy
import random

from std_msgs.msg import Float32
from eod_pkg.srv import VelRuota,VelRuotaRequest,VelRuotaResponse

class Pid_Controller_Sx():

	def __init__(self):
		rospy.init_node('pid_controller_sx')

		# Parmetri PID
		self.Kp = rospy.get_param('~Kp',5.102)
		self.Ki = rospy.get_param('~Ki',6.2232)
		self.Kd = rospy.get_param('~Kd',0)

		# Limite azione controllo
		self.v_min = rospy.get_param('~v_min',0.8)
		self.v_max = rospy.get_param('~v_max',7.0)

		# Inizializzazione variabili interne
		self.ref = 0.0
		self.ref_old = 0.0
		self.vel = 0.0
		self.err = 0.0
		self.err_old = 0.0
		self.v_input = 0.0
		self.integrale = 0.0
		self.derivata = 0.0
		self.t_old = rospy.Time.now()

		# Frequenza di campionamento
		self.fc = rospy.get_param('~fc',10)
		self.rc = rospy.Rate(self.fc)

		# Soglia variazione riferimento
		self.eps = 1;

		# Servizi
		rospy.wait_for_service('vel_encoder_sx')
		self.encoder = rospy.ServiceProxy('vel_encoder_sx',VelRuota)
		
		# Topic
		rospy.Subscriber('vel_ref', Float32, self.ref_callback) 
		self.pub_motor = rospy.Publisher('motor_cmd_sx', Float32, queue_size=10)
		self.pub_vel_calcolata = rospy.Publisher('velociraptor', Float32, queue_size=10)

	def spin(self):

		while not rospy.is_shutdown():
			# Calcolo dt
			self.dt = (rospy.Time.now() - self.t_old).to_sec()
			self.t_old = rospy.Time.now()

			# Lettura encoder
			self.vel = (self.encoder(self.dt)).vel
			print self.vel			
			# Calcolo errore
			self.err = self.ref - self.vel
			print "Vel",self.vel
			print "Err",self.err
			print "---"
			self.pub_vel_calcolata.publish(self.vel)
			

			# Calcolo azione di controllo
			self.pid()

			# Pubblica azione di controllo
			#print(self.v_input)
			self.pub_motor.publish(self.v_input)

			# Attesa prossimo istante di campionamento
			self.rc.sleep()

	def pid(self):

		if(abs(self.ref - self.ref_old) > self.eps):
			self.integrale = 0.0
			self.derivata = 0.0

		if(self.ref == 0.0):
			self.v_input = 0.0
			self.integrale = 0.0
			self.derivata = 0.0
			return

		self.integrale = self.integrale + (self.err * self.dt)
		print "Int", self.integrale
		print "<<<<"
		self.derivata = (self.err - self.err_old)/self.dt
		self.err_old = self.err

		self.v_input =  (self.Kp * self.err) + (self.Ki * self.integrale) + (self.Kd * self.derivata)

		print "V in", self.v_input

		try:
			sign = self.v_input/abs(self.v_input)
		except ZeroDivisionError:
			sign = 1

		# Saturatore
		if self.ref > 0.0:
			#movimento in avanti
			if self.v_input > self.v_max:
				print "SAT1"
				self.v_input = self.v_max
				self.integrale = self.integrale - (self.err * self.dt)
			elif self.v_input < self.v_min:
				print "SAT2"
				self.v_input = self.v_min
				#self.integrale = self.integrale - (self.err * self.dt)
		else:
			if self.v_input > -1.0*self.v_min:
				print "SAT3"
				self.v_input = -1.0*self.v_min
				#self.integrale = self.integrale - (self.err * self.dt)
			elif self.v_input < -1.0*self.v_max:
				print "SAT4"
				self.v_input = -1.0*self.v_max
				self.integrale = self.integrale - (self.err * self.dt)

	def ref_callback(self, msg):
		self.ref_old = self.ref
		self.ref = msg.data


if __name__ == '__main__':
	pid_controller = Pid_Controller_Sx()
	pid_controller.spin()
