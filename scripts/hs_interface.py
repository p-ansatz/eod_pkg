#!/usr/bin/env python

import rospy
import numpy as np
from math import pi

from std_msgs.msg import Float32
from eod_pkg.msg import Ticks
from eod_pkg.srv import VelRuota,VelRuotaResponse,VelRuotaRequest

from gpiozero import Motor
import RPi.GPIO as GPIO

tick_dx = 0.0 # Contatore tick encoder ruota destra - per PID
tick_sx = 0.0 # Contatore tick encoder ruota sinistra - per PID
odom_tick_dx = 0 # Contatore tick encoder ruota destra - per odometria
odom_tick_sx = 0 # Contatore tick encoder ruota destra - per odometria

class HS_Interface():

	def __init__(self):
		rospy.init_node('hs_interface')

		# Parametri
		self.n_tick = rospy.get_param('~n_tick',20) # Numero tick encoder
		self.r = rospy.get_param('~r',0.031) # Raggio ruota
		self.v_min = rospy.get_param('~v_min',0.8) # Tensione minima motore
		self.v_max = rospy.get_param('~v_max',7.0) # Tensione massima motore
		self.n_vel_array = rospy.get_param('~n_vel_array',5) # Dimensione vettore velocita' media

		# Inizializzazione variabili interne
		self.dir_dx = 1 # Direzione rotazione ruota destra [1=avanti]
		self.dir_sx = 1 # Direzione rotazione ruota sinistra [1=avanti]
		self.vel_array_dx = [0.0]*self.n_vel_array
		self.vel_array_sx = [0.0]*self.n_vel_array
		self.pesi = range(1,self.n_vel_array+1)

		# Frequenza di campionamento
		self.fc = rospy.get_param('~fc',0.01)
		self.rc = rospy.Rate(self.fc)

		# Pin Encoder
		self.pin_enc_dx = 2
		self.pin_enc_sx = 3

		# Setup encoder Destro
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.pin_enc_dx, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
		GPIO.add_event_detect(self.pin_enc_dx, GPIO.RISING, callback=callback_enc_dx)

		# Setup encoder Sinistro
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.pin_enc_sx, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
		GPIO.add_event_detect(self.pin_enc_sx, GPIO.RISING, callback=callback_enc_sx)

		# Pin Motore
		self.pin_motor_dx_1 = 19
		self.pin_motor_dx_2 = 26
		self.pin_motor_sx_1 = 6
		self.pin_motor_sx_2 = 13

		# Setup motore Destro
		self.motore_dx = Motor(self.pin_motor_dx_1 ,self.pin_motor_dx_2, True)

		# Setup motore Sinistro
		self.motore_sx = Motor(self.pin_motor_sx_1 ,self.pin_motor_sx_2,True)

		# Servizi
		service_dx = rospy.Service('vel_encoder_dx', VelRuota, self.lettura_encoder_dx)	
		service_sx = rospy.Service('vel_encoder_sx', VelRuota, self.lettura_encoder_sx)

		# Topic
		rospy.Subscriber('motor_cmd_dx', Float32, self.motor_cmd_dx_callback)
		rospy.Subscriber('motor_cmd_sx', Float32, self.motor_cmd_sx_callback)
		self.pub_tick = rospy.Publisher('tick_ruote', Ticks, queue_size=10)

	def spin(self):
		global tick_dx
		global tick_sx

		while not rospy.is_shutdown():
			self.rc.sleep()

	def lettura_encoder_dx(self,request):
		global odom_tick_dx
		global odom_tick_sx
		global tick_dx

		dt = request.dt
		print tick_dx
		m = (tick_dx*(2*pi*self.r))/self.n_tick
		vel = m/dt
		tick_dx = 0		
		self.vettore=[]
		self.vel_array_dx.pop(0)
		self.vel_array_dx.append(vel)
		#g = (self.n_vel_array*(self.n_vel_array-1))/2
		#for i in range(0,self.n_vel_array):
		#	self.vettore[i] = (self.vel_array_dx[i] * self.pesi[i])/g
		#
		msg_tick = Ticks()
		msg_tick.dx = odom_tick_dx
		msg_tick.sx = odom_tick_sx
		self.pub_tick.publish(msg_tick)

		odom_tick_dx = 0
		odom_tick_sx = 0

		vel = np.mean(self.vel_array_dx)
	

		return VelRuotaResponse(vel*self.dir_dx)

	def lettura_encoder_sx(self,request):
		global odom_tick_dx
		global odom_tick_sx
		global tick_sx

		dt = request.dt
		print tick_sx
		m = (tick_sx*(2*pi*self.r))/self.n_tick
		vel = m/dt
		tick_sx = 0
		self.vel_array_sx.pop(0)
		self.vel_array_sx.append(vel)
		
		#
		msg_tick = Ticks()
		msg_tick.dx = odom_tick_dx
		msg_tick.sx = odom_tick_sx

		self.pub_tick.publish(msg_tick)
		odom_tick_dx = 0
		odom_tick_sx = 0

		vel = np.mean(self.vel_array_sx)

		return VelRuotaResponse(vel*self.dir_sx)

	def motor_cmd_dx_callback(self,msg):
		v_input = msg.data
		
		try:
			self.dir_dx = v_input/abs(v_input)
		except ZeroDivisionError:
			self.dir_dx = 1

		v_input = abs(v_input/self.v_max)

		if (self.dir_dx < 0 ):
			self.motore_dx.backward(v_input)
		else:
			self.motore_dx.forward(v_input)
	
	def motor_cmd_sx_callback(self,msg):
		v_input = msg.data
		
		try:
			self.dir_sx = v_input/abs(v_input)
		except ZeroDivisionError:
			self.dir_sx = 1

		v_input = abs(v_input/self.v_max)

		if (self.dir_sx < 0 ):
			self.motore_sx.backward(v_input)
		else:
			self.motore_sx.forward(v_input)


def callback_enc_dx(channel):
	global tick_dx
	global odom_tick_dx

	tick_dx = tick_dx+1
	odom_tick_dx = odom_tick_dx+1

def callback_enc_sx(channel):
	global tick_sx
	global odom_tick_sx

	tick_sx = tick_sx+1
	odom_tick_sx = odom_tick_sx+1


if __name__ == '__main__':
	hs = HS_Interface()
	hs.spin()
