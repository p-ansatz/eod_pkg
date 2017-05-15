#!/usr/bin/env python
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction
    
waypoints = [ ['one', (1.0, 1.5), (0.0, 0.0, 0.0, 1.0)]]
target_point = [ (0.0, 0.0, 0.0) , (0.0, 0.0 ,0.0, 0.0)]
move_client  = actionlib.SimpleActionClient('move_base', MoveBaseAction)
target_flag = False

def init():
    global waypoints
    global target_point
    global move_client    

    move_client  = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    waypoints = [ ['one', (1.0, 1.5), (0.0, 0.0, 0.0, 1.0)]
    # ,['two', (1.0, 2.5), (0.0, 0.0, 0.0, 1.0)] 
    ]

def abort_move():
    global move_client
    print "cancel goal"
    move_client.cancel_all_goals()

# TARGET POINT
def get_target_point():
    global target_point
    return target_point

def modify_target_point(tp):
    global target_point
    target_point = [ (tp[0], tp[1], tp[2]) , (tp[3], tp[4] ,tp[5], tp[6])]
    print target_point