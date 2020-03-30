import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

from math import pow, atan2, sqrt, pi

def euclidean_distance(source, target):
		return sqrt(pow((target.x - source.x), 2) +
					pow((source.y - target.y), 2))

def min_angle_diff(angle, target_angle):
	delta = target_angle - angle
	if abs(delta) >= pi:
		module = 2*pi - abs(delta)
		if delta > 0:
			min_delta = -1. * module
		else:
			min_delta = module
	else:
		min_delta=delta
	return min_delta

def to_positive_angle(angle):
	return (angle+2*pi)%(2*pi) 
