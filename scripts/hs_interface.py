#!/usr/bin/env python

import rospy

from std_msgs.msg import Byte
from eod_pkg.msg import Wheels_Vel

from gpiozero import Motor
import RPi.GPIO as GPIO

# ------ VARIABILI GLOBALI------        
dir_dx = 1
dir_sx = 1

# ------ TOPIC GLOBALI------
tick_dx_pub = rospy.Publisher('tick_dx', Byte, queue_size=10)
tick_sx_pub = rospy.Publisher('tick_sx', Byte, queue_size=10)

class HS_interface():

    def __init__(self):
        rospy.init_node('hs_interface')

	# ------ PARAMETRI ------
	# frequenza del loop 
        self.freq = rospy.get_param('/eod/loop_freq/default')
        self.rate = rospy.Rate(self.freq)

	# Pin Encoder
        self.pin_enc_dx = rospy.get_param('/eod/pin/pin_enc_dx')
        self.pin_enc_sx = rospy.get_param('/eod/pin/pin_enc_sx')

	# Setup encoder Destro
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_enc_dx, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.pin_enc_dx, GPIO.RISING, callback=callback_enc_dx)

        # Setup encoder Sinistro
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_enc_sx, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.pin_enc_sx, GPIO.RISING, callback=callback_enc_sx)

        # Pin Motore
        self.pin_motor_dx_f = rospy.get_param('/eod/pin/pin_motor_dx_f')
        self.pin_motor_dx_b = rospy.get_param('/eod/pin/pin_motor_dx_b')
        self.pin_motor_sx_f = rospy.get_param('/eod/pin/pin_motor_sx_f')
        self.pin_motor_sx_b = rospy.get_param('/eod/pin/pin_motor_sx_b')

        # Setup motore Destro
        self.motor_dx = Motor(self.pin_motor_dx_f, self.pin_motor_dx_b, True)

        # Setup motore Sinistro
        self.motor_sx = Motor(self.pin_motor_sx_f, self.pin_motor_sx_b, True)

        # ------ TOPIC ------
        rospy.Subscriber('motor_cmd', Wheels_Vel, self.motor_cmd_callback)


    def loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

    def motor_cmd_callback(self, msg):
        global dir_dx
        global dir_sx
        
        v_input_dx = msg.right
        v_input_sx = msg.left

        if v_input_dx >= 0:
            self.motor_dx.forward(v_input_dx)
            dir_dx = 1
        else:
            self.motor_dx.backward(-v_input_dx)
            dir_dx = -1

        if v_input_sx >= 0:
            self.motor_sx.forward(v_input_sx)
            dir_sx = 1
        else:
            self.motor_dx.backward(-v_input_sx)
            dir_sx = -1
            
# ------ CALLBACK ENCODER ------
def callback_enc_dx(channel):
    global dir_dx
    global tick_dx_pub
    
    enc_msg = Byte()
    enc_msg.data = dir_dx
    tick_dx_pub.publish(enc_msg)


def callback_enc_sx(channel):
    global dir_sx
    global tick_sx_pub
    
    enc_msg = Byte()
    enc_msg.data = dir_sx
    tick_sx_pub.publish(enc_msg)


if __name__ == '__main__':
	hs = HS_interface()
	hs.loop()            
