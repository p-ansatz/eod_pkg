#!/usr/bin/env python
import rospy
import tf
import math

quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0 ,0.0)
print quaternion
quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0 ,math.pi/2.0)
print quaternion
quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0 ,math.pi)
print quaternion
quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0 ,-math.pi/2.0)
print quaternion

