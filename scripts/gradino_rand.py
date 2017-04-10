#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Float32
from eod_pkg.srv import VelRuota,VelRuotaRequest,VelRuotaResponse

rospy.init_node('questo')

fc = rospy.get_param('~fc',10)
rc = rospy.Rate(fc)

t_old = rospy.Time.now()

rospy.wait_for_service('vel_encoder_dx')
encoder = rospy.ServiceProxy('vel_encoder_dx',VelRuota)

pub = rospy.Publisher('motor_cmd_dx', Float32, queue_size=10)

i=1.5
while not rospy.is_shutdown():

	pub.publish(i)
	
	dt = (rospy.Time.now() - t_old).to_sec()
	t_old = rospy.Time.now()

	# Lettura encoder
	vel = (encoder(dt)).vel
	print vel

	rc.sleep()
