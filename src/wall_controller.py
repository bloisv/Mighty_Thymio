import rospy
import movement_utils as mv
from pid import PID
from geometry_msgs.msg import Twist, Pose
from math import pi


class WallController:
	def __init__(self, proximity_threshold):
		self.proximity_threshold = proximity_threshold
		self.motion_controller = mv.ToTargetPController(linear_speed=2.,orientation_speed = 2.)

		self.proximity_min_range = 0.010
		self.proximity_max_range = 0.12
		self.WALL_REACHED = False
		self.DONE = False
		self.ROTATING = False
		self.ROTATED = False
		self.target_orientation = pi
		self.theta_pid = PID(Kp=.4,Ki= 0.,Kd = 0.4)

	def run(self, proximity, position, orientation):
		velocity = Twist()
		velocity.linear.x = 0
		velocity.linear.y = 0
		velocity.linear.z = 0
		velocity.angular.x = 0
		velocity.angular.y = 0
		velocity.angular.z = 0.

		# print("Orientation: {0}\n".format(orientation))
		if not self.WALL_REACHED:
			rospy.loginfo(proximity[2])

	
		if proximity[2] > 0.11:
			velocity.linear.x = .15
		elif proximity[2] > 0.05:
			velocity.linear.x = 0.033
		else:
			velocity.linear.x = 0.
			self.WALL_REACHED = True

		if self.WALL_REACHED and not self.ROTATING:
			self.ROTATING=True
			self.target_orientation = (orientation+pi)%(2*pi)


		if not self.ROTATED and self.WALL_REACHED:
			print("Rear proximity sensors values: lx->{0}---{1}<-rx".format(proximity[5],proximity[6]))
			print("Orientation: {0}\n".format(orientation))
			done , velocity = self.motion_controller.move(position,orientation,
				position,target_orientation=self.target_orientation
			)
			if done:
				self.ROTATED = True

		if not self.DONE and self.ROTATED:
			error = proximity[6]-proximity[5] 
			target = self.theta_pid.step(90* error,dt=100)
			print(target)
			done , velocity = self.motion_controller.move(position,orientation,
				position,target_orientation=-target+self.target_orientation
			)
			if done:
				self.DONE = True

			
		return velocity





