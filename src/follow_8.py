#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import movement_utils as mv
from math import pi




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
		

	def run(self):
		velocity = Twist()
		velocity.linear.y = 0
		velocity.linear.z = 0
		velocity.angular.x = 0
		velocity.angular.y = 0

		velocity.angular.z = 0.2
		velocity.linear.x = 0


		speed = 0.33
		radius = 0.5
		positive_orientation = True
		started_8 = False
		finishing_8 = False
		half_8 = False
		espilon = 0.1

		while not rospy.is_shutdown():
			

			if abs(self.orientation - pi/2) < espilon:
				started_8 = True
			if started_8 and not finishing_8 and abs(self.orientation) < espilon:
				positive_orientation = False
				half_8 = True

			if half_8 and abs(self.orientation - 3.*pi/2.) < espilon:
				finishing_8 = True

				
			if  finishing_8 and abs(self.orientation) < espilon:
				positive_orientation = True
				started_8 = False
				finishing_8 = False

			# Debug
			# print("Started: {0}, Half: {1}, Finished: {2}\n".format(started_8, half_8, finishing_8))

			velocity = mv.circular_motion(speed=speed,
			 radius=radius,
			 positive_orientation=positive_orientation)
			
			self.velocity_publisher.publish(velocity)
			self.rate.sleep()

if __name__ == '__main__':
	try:
		Thymio_controller = Thymio_Follow_8_Controller('thymio10')
		Thymio_controller.run()
	except rospy.ROSInterruptException as e:
		pass
