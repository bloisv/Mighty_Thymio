import rospy
import movement_utils as mv
from geometry_msgs.msg import Twist, Pose
from math import pi


class WallController:
	def __init__(self, proximity_threshold):
		self.proximity_threshold = proximity_threshold
		self.motion_controller = mv.ToTargetPController(linear_speed=2.,orientation_speed = 1.5)

		self.proximity_min_range = 0.010
		self.proximity_max_range = 0.12
		self.WALL_REACHED = False
		self.DONE = False

	def run(self, proximity, position, orientation):
		velocity = Twist()
		velocity.linear.y = 0
		velocity.linear.z = 0
		velocity.angular.x = 0
		velocity.angular.y = 0
		velocity.angular.z = 0.


		rospy.loginfo(proximity[2])

		
		if proximity[2] > 0.11:
			velocity.linear.x = .15
		elif proximity[2] > 0.8:
			velocity.linear.x = 0.033
		else:
			velocity.linear.x = 0.
			self.WALL_REACHED = True


		# Turn 180 degrees ---- buggy
		if not self.DONE and self.WALL_REACHED:
			target_orientation = (orientation+pi)%(2*pi)
			done , velocity = self.motion_controller.move(position,orientation,
				position,target_orientation=target_orientation
			)
			if done:
				self.DONE = True
			
		return velocity





