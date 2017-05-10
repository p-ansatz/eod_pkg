#!/usr/bin/env python
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction

def init():
    global waypoints
    global target_point # Punto, in prossimità dell'oggetto, nel quale si deve portare il robot
    global move_client

    move_client  = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    waypoints = [ ['one', (2.1, 2.2), (0.0, 0.0, 0.0, 1.0)],['two', (6.5, 4.43), (0.0, 0.0, -0.984047240305, 0.177907360295)] ]
    target_point = [ (0.0, 0.0, 0.0) , (0.0, 0.0 ,0.0, 0.0)]


def abort_move():
	global move_client

	move_client.cancel()

# TARGET POINT
def get_target_point():
	global target_point

	return target_point

def modify_target_point(tp):
	global target_point

	target_point = [ (tp[0], tp[1], tp[2]) , (tp[3], tp[4] ,tp[5], tp[6])]
