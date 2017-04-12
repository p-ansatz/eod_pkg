#!/usr/bin/env python

import rospy
import numpy as np

from math import pi

from std_msgs.msg import Byte
from std_msgs.msg import Float32

from eod_pkg.msg import Wheels_Vel

class WheelsVelSensor():
    
    def __init__(self):
        rospy.init_node('wheels_vel_sensor')

        # ------ PARAMETRI ------
        # frequenza del loop 
        self.freq = rospy.get_param('/eod/loop_freq/wheels_vel')
        self.rate = rospy.Rate(self.freq)

        # parametri fisici
        self.r = rospy.get_param('/eod/physics/r', 0.013) # dimensione ruote
        self.tick_encoder = rospy.get_param('/eod/physics/n_tick', 20) # numero tick encoder

        # dimensione dei vettori fifo
        self.n = 5

        # ------ VARIABILI ------
        self.tick_dx = 0
        self.tick_sx = 0

        self.fifo_dx = [0.0]*self.n  	# vettore contenente i tick_dx registrati negli ultimi n cicli
        self.fifo_sx = [0.0]*self.n 	# vettore contenente i tick_sx registrati negli ultimi n cicli

        self.t_old = rospy.Time.now() # tempo all'ultima misurazione

        self.vel_dx = 0
        self.vel_sx = 0

        # ------ TOPIC ------
        # sui topic tick_dx e tick_sx il valore 1 indica un tick misurato
        # in senso orario, -1 in senso antiorario
        rospy.Subscriber('tick_dx', Byte, self.tick_dx_callback)
        rospy.Subscriber('tick_sx', Byte, self.tick_sx_callback)

        self.pub_vel = rospy.Publisher('wheels_velocity', Wheels_Vel, queue_size=10)

    def loop(self):

        while not rospy.is_shutdown():

            self.update_measurement()
            
            self.compute_velocity()

            self.publish_velocity()

            self.rate.sleep()


    def update_measurement(self):
        # aggiornamento vettori misure
        self.fifo_dx.pop(0)
        self.fifo_dx.append(self.tick_dx)
        self.fifo_sx.pop(0)
        self.fifo_sx.append(self.tick_sx)

        # azzermento numero di tick contati
        self.tick_dx = 0
        self.tick_sx = 0

    def compute_velocity(self):
        # media del numero di tick negli ultimi n cicli
        mean_dx = np.mean(self.fifo_dx)
        mean_sx = np.mean(self.fifo_sx)

        # distanze in metri
        m_dx = (mean_dx*(2*pi*self.r))/self.tick_encoder
        m_sx = (mean_sx*(2*pi*self.r))/self.tick_encoder

        # tempo trascorso dall'ultima misura
        t = rospy.Time.now()
        dt = (t - self.t_old).to_sec()
        self.t_old = t

        # velocita' in m/s
        self.vel_dx = m_dx/dt
        self.vel_sx = m_sx/dt

    def publish_velocity(self):
        # pubblicazione sul topic eod/wheels_velocity
        msg_vel = Wheels_Vel()
        msg_vel.right = self.vel_dx
        msg_vel.left = self.vel_sx
        self.pub_vel.publish(msg_vel)

    def tick_dx_callback(self, msg):
        self.tick_dx += msg.data
        print("DX %d" % self.tick_dx)

    def tick_sx_callback(self, msg):
        self.tick_sx += msg.data
        print("SX %d" % self.tick_sx)
        
if __name__ == '__main__':
    wvs = WheelsVelSensor()
    wvs.loop()
