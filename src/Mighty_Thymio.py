#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import movement_utils as mv
from wall_controller import WallController
from follow_8_controller import Follow8Controller
from math import pi




class Mighty_Thymio:
	def __init__(self, thymio_name):
		# self.mode = 'FOLLOW8'
		self.mode = 'AVOID_WALL'
		rospy.init_node('Mighty_Thymio', anonymous=True)
		self.rate = rospy.Rate(10)

		
		self.velocity_publisher = rospy.Publisher('/'+thymio_name+'/cmd_vel', Twist, queue_size=10)
		self.odometry_subscriber = rospy.Subscriber('/'+thymio_name+'/odom', Odometry, self.update_pose)
		
		prx = '/'+thymio_name+'/proximity/'
		self.proximity_sensors_name = [
		 prx+'left',
		 prx+'center_left', 
		 prx+'center', 
		 prx+'center_right',
		 prx+'right'
		]

		callback = [self.update_proximity_left,
		self.update_proximity_center_left,
		self.update_proximity_center,
		self.update_proximity_center_right,
		self.update_proximity_right
		]

		self.proximity_subscribers = [
			rospy.Subscriber(sensor, Range, callback[i]) 
			for i,sensor in enumerate(self.proximity_sensors_name)
		]

		self.position = Point()
		self.orientation = 0
		self.proximity = [0.12] * 5


	def update_pose(self, data):
		self.position = data.pose.pose.position
		quaternion = data.pose.pose.orientation
		explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
		_,_,self.orientation = euler_from_quaternion(explicit_quat)
		self.orientation = mv.to_positive_angle(self.orientation)

	def update_proximity_left(self, data):
		self.proximity[0] = data.range

	def update_proximity_center_left(self, data):
		self.proximity[1] = data.range
	
	def update_proximity_center(self, data):
		self.proximity[2] = data.range

	def update_proximity_center_right(self, data):
		self.proximity[3] = data.range

	def update_proximity_right(self, data):
		self.proximity[4] = data.range
		

	def run(self):

		if self.mode == 'FOLLOW8':
			self.controller = Follow8Controller()
		else:
			self.controller = WallController(0.2)
		

		while not rospy.is_shutdown():
			if self.mode == 'FOLLOW8':
				vel = self.controller.run(self.orientation) 
			else:
				vel = self.controller.run(self.proximity, self.position, self.orientation)
			
			self.velocity_publisher.publish(vel)
			self.rate.sleep()


if __name__ == '__main__':
	try:
		Thymio_controller = Mighty_Thymio('thymio10')
		Thymio_controller.run()
	except rospy.ROSInterruptException as e:
		pass
