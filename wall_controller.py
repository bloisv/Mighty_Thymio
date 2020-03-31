import movement_utils as mv
from geometry_msgs.msg import Twist, Pose

class WallController:
	def __init__(self, proximity_threshold, pub, t):
		self.proximity_threshold = proximity_threshold
		self.motion_controller = mv.ToTargetPController(linear_speed=2.,orientation_speed = 1.5)

		self.proximity_min_range = 0.010
		self.proximity_max_range = 0.12

		self.vel_msg = Twist()
		self.vel_msg.linear.x = 0
		self.vel_msg.linear.y = 0
		self.vel_msg.linear.z = 0
		self.vel_msg.angular.x = 0
		self.vel_msg.angular.y = 0
		self.vel_msg.angular.z = 0
		self.velocity_publisher = pub

	def run(self, proximity):

		# rospy.loginfo(proximity[2])
		self.vel_msg.linear.x = 0.05

		self.velocity_publisher.publish(vel_msg)
		#if proximity[2] > 0.10:
		#	velocity.linear.x = .005
		#else
		#	velocity.linear.x = 0.
		return self.vel_msg





