#!/usr/bin/env python

def init():
    global waypoints
    global target_point
    global object_point

    waypoints = [ ['one', (2.1, 2.2), (0.0, 0.0, 0.0, 1.0)],['two', (6.5, 4.43), (0.0, 0.0, -0.984047240305, 0.177907360295)] ]
    target_point = [ (0.0, 0.0) , (0.0, 0.0 ,0.0, 0.0)]
    object_point = [ (0.0, 0.0) , (0.0, 0.0 ,0.0, 0.0)]
