
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import movement_utils as mv
from math import pi


class Follow8Controller:
	def __init__(self, speed=0.33, radius=0.5, debug=True):
		self.speed = speed
		self.radius = radius
		self.positive_orientation = True
		self.started_8 = False
		self.finishing_8 = False
		self.half_8 = False
		self.espilon = 0.05

		self.debug = True

	def run(self, orientation):

		if abs(orientation - pi/2) < self.espilon:
			self.started_8 = True
		if self.started_8 and not self.finishing_8 and abs(orientation) < self.espilon:
			self.positive_orientation = False
			self.half_8 = True

		if self.half_8 and abs(orientation - 3.*pi/2.) < self.espilon:
			self.finishing_8 = True

			
		if  self.finishing_8 and abs(orientation) < self.espilon:
			self.positive_orientation = True
			self.started_8 = False
			self.half_8 = False
			self.finishing_8 = False

		if self.debug:
			print("Sterted 8: {0}; Half 8 {1}; Finishing_8 {2}"
				.format(self.started_8,self.half_8,self.finishing_8))

		velocity = mv.circular_motion(speed=self.speed,
		 radius=self.radius,
		 positive_orientation=self.positive_orientation)

		return velocity
