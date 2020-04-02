import rospy
import movement_utils as mv
from pid import PID
from geometry_msgs.msg import Twist, Pose, Point
from math import pi, cos, sin
from collections import deque


class WallController:
	def __init__(self, proximity_threshold, debug=False):
		self.proximity_min_range = 0.010
		self.proximity_max_range = 0.12
		self.WALL_REACHED = False
		self.PERPENDICULAR = False
		self.ROTATING = False
		self.ROTATED = False
		self.MAX_DIST = False
		self.DONE = False

		self.proximity_threshold = proximity_threshold
		self.motion_controller = mv.ToTargetPController(linear_speed=0.20,orientation_speed = 2.5)
		self.debug = debug
		self.target_orientation = pi/2.


		self.theta_pid = PID(Kp=.2,Ki= 0.,Kd = 0.)
		#self.error_buffer = deque(maxlen=8)


	def run(self, proximity, position, orientation):
		velocity = Twist()
		velocity.linear.x = 0
		velocity.linear.y = 0
		velocity.linear.z = 0
		velocity.angular.x = 0
		velocity.angular.y = 0
		velocity.angular.z = 0.

		
		if not self.WALL_REACHED:
			if proximity[2] > 0.11:
				velocity.linear.x = .15
			elif proximity[2] > 0.05:
				velocity.linear.x = 0.033
			else:
				velocity.linear.x = 0.
				self.WALL_REACHED = True
				print("Wall reached. Turning in place...")

		if self.WALL_REACHED and not self.PERPENDICULAR:
			max_orientation_speed = 0.75
			angular_z = 20. * (proximity[4] - proximity[0])
			module = min(abs(angular_z), max_orientation_speed)
			angular_z *= module / abs(angular_z)
			velocity.angular.z = angular_z

			print(angular_z)
			if abs(angular_z) < 0.01:
				self.PERPENDICULAR=True
				self.target_orientation = (orientation+pi)%(2*pi)


		if not self.ROTATED and self.PERPENDICULAR:
			if self.debug:
				print("Rear proximity sensors values: lx->{0}---{1}<-rx".format(proximity[5],proximity[6]))
				print("Orientation: {0}\n".format(orientation))

			done , velocity = self.motion_controller.move(position,orientation,
				position,target_orientation=self.target_orientation,
				max_orientation_speed=.75
			)

			if done:
				self.ROTATED = True
				print("Done. Moving 2 meters away from the wall...")

		if not self.MAX_DIST and self.ROTATED:
			if proximity[5] < 0.1199:

				velocity.linear.x = 0.1
			else:
				velocity.linear.x = 0.
				self.final_target = Point()
				distance = 2. - 0.12 - 0.03
				self.final_target.y = position.y + sin(orientation) * distance
				self.final_target.x = position.x + cos(orientation) * distance
				self.MAX_DIST = True


		if not self.DONE and self.MAX_DIST:
				done , velocity = self.motion_controller.move(position,orientation,
					self.final_target, max_linear_speed = 0.2
				)
				if self.debug:
					print(velocity)
					print("{0} --> {1}".format(position.x,self.final_target.x))
				if done:
					print("Done. Final position: ({0},{1})".format(position.x,position.y))
					self.DONE = True

		if self.DONE:
			velocity.linear.x = 0.
			
		return velocity





