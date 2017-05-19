#!/usr/bin/env python
import rospy
import tf
from math import pi

quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0 ,0.0)
print quaternion
quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0 ,pi/2.0)
print quaternion
quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0 ,pi)
print quaternion
quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0 ,-pi/2.0)
print quaternion

