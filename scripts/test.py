#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32
from eod_pkg.msg import Wheels_Vel


def vel_callback(msg):
	print "DX %f"%msg.right
	print "SX %f"%msg.left

rospy.init_node('ide')

fc = rospy.get_param('~fc',1)
rc = rospy.Rate(fc)

rospy.Subscriber('wheels_velocity', Wheels_Vel, vel_callback)

cmd_pub = rospy.Publisher('v_target', Wheels_Vel, queue_size=10)

input = 0.3

if __name__ == '__main__':
   while not rospy.is_shutdown():

   	time.sleep(1)

	cmd_pub.publish(input,input)
	
	time.sleep(8)

	cmd_pub.publish(0.0,0.0)
	
	break
