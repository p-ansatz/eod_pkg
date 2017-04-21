#!/usr/bin/env python

import rospy



class HS_interface():
	"""docstring for HS_interface"""
	
	def __init__(self):
		rospy.init_node('eod/hs_interface')

		# ------ PARAMETRI ------
		# frequenza del loop 
		self.freq = rospy.get_param('eod/loop_freq/default', 0.1)
		self.rate = rospy.Rate(self.freq)

		# Pin Encoder
		self.pin_enc_dx = rospy.get_param('eod/pin/pin_enc_dx')
		self.pin_enc_sx = rospy.get_param('eod/pin/pin_enc_sx')

		# Setup encoder Destro
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.pin_enc_dx, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
		GPIO.add_event_detect(self.pin_enc_dx, GPIO.RISING, callback=callback_enc_dx)

		# Setup encoder Sinistro
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.pin_enc_sx, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
		GPIO.add_event_detect(self.pin_enc_sx, GPIO.RISING, callback=callback_enc_sx)

		# Pin Motore
		self.pin_motor_dx_f = rospy.get_param('eod/pin/pin_motor_dx_f')
		self.pin_motor_dx_b = rospy.get_param('eod/pin/pin_motor_dx_b')
		self.pin_motor_sx_f = rospy.get_param('eod/pin/pin_motor_sx_f')
		self.pin_motor_sx_b = rospy.get_param('eod/pin/pin_motor_sx_b')

		# Setup motore Destro
		self.motor_dx = Motor(self.pin_motor_dx_f, self.pin_motor_dx_b, True)

		# Setup motore Sinistro
		self.motor_sx = Motor(self.pin_motor_sx_f, self.pin_motor_sx_b, True)

		# ------ VARIABILI ------
		self.dir_dx = 1
		self.dir_sx = 1

		# ------ TOPIC ------
		rospy.Subscriber('eod/motor_cmd', Wheels_Vel, self.motor_cmd_callback)

		self.tick_dx_pub = rospy.Publisher('eod/tick_dx', Byte, queue_size=10)
		self.tick_sx_pub = rospy.Publisher('eod/tick_sx', Byte, queue_size=10)

	def loop(self)
		while not rospy.is_shutdown():

			self.rate.sleep()

	def callback_enc_dx(self):
		enc_msg = Byte()
		enc_msg.data = self.dir_dx
		tick_dx_pub.publish(enc_msg)


	def callback_enc_sx(self):
		enc_msg = Byte()
		enc_msg.data = self.dir_sx
		tick_sx_pub.publish(enc_msg)

	def motor_cmd_callback(self, msg):
		v_input_dx = msg.right
		v_input_sx = msg.left

		if v_input_dx >= 0:
			self.motor_dx.forward(v_input_dx)
			self.dir_dx = 1
		else:
			self.motor_dx.backward(-v_input_dx)
			self.dir_dx = -1

		if v_input_sx >= 0:
			self.dir_sx = 1
		else:
			self.motor_dx.backward(-v_input_dx)
			self.dir_sx = -1
