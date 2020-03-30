#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import movement_utils as mv




class Thymio_Follow_8_Controller:
	def __init__(self, thymio_name):
		rospy.init_node('thmyio_follow_8', anonymous=True)
		self.rate = rospy.Rate(10)

		self.velocity_publisher = rospy.Publisher('/'+thymio_name+'/cmd_vel', Twist, queue_size=10)
		self.odometry_subscriber = rospy.Subscriber('/'+thymio_name+'/odom', Odometry, self.update_pose)
		self.position = Point()
		self.orientation = 0

	def update_pose(self, data):
		self.position = data.pose.pose.position
		quaternion = data.pose.pose.orientation
		explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
		_,_,self.orientation = euler_from_quaternion(explicit_quat)
		self.orientation = mv.to_positive_angle(self.orientation)



	def circular_motion(self):
		rospy.loginfo(mv.euclidean_distance(Point(),Point()))

	def run(self):
		velocity = Twist()
		velocity.linear.y = 0
		velocity.linear.z = 0
		velocity.angular.x = 0
		velocity.angular.y = 0

		velocity.angular.z = 0.2
		velocity.linear.x = 0
		while not rospy.is_shutdown():
			self.velocity_publisher.publish(velocity)
			#self.circular_motion()
			rospy.loginfo(self.orientation)
			rospy.loginfo(mv.min_angle_diff(self.orientation, 1.))
			self.rate.sleep()

if __name__ == '__main__':
	try:
		Thymio_controller = Thymio_Follow_8_Controller('thymio10')
		Thymio_controller.run()
	except rospy.ROSInterruptException as e:
		pass
