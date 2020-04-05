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
from explorer_controller import ExplorerController
from math import pi


class Mighty_Thymio:
    def __init__(self, ):
        rospy.init_node('Mighty_Thymio', anonymous=True)
        self.rate = rospy.Rate(10)

        self.thymio_name = rospy.get_param("~name")

        mode_num = rospy.get_param('~mode')
        if mode_num == 0:
        	self.mode = 'FOLLOW8'
        elif mode_num == 1:
        	self.mode = 'AVOID_WALL'
        else:
        	self.mode = 'EXPLORER'
    

        print("Process for "+self.thymio_name+" has started!")

        self.velocity_publisher = rospy.Publisher('/' + self.thymio_name + '/cmd_vel', Twist, queue_size=10)
        self.odometry_subscriber = rospy.Subscriber('/' + self.thymio_name + '/odom', Odometry, self.update_pose)

        prx = '/' + self.thymio_name + '/proximity/'
        self.proximity_sensors_name = [
            prx + 'left',
            prx + 'center_left',
            prx + 'center',
            prx + 'center_right',
            prx + 'right',
            prx + 'rear_left',
            prx + 'rear_right'
        ]

        callback = [self.update_proximity_left,
                    self.update_proximity_center_left,
                    self.update_proximity_center,
                    self.update_proximity_center_right,
                    self.update_proximity_right,
                    self.update_proximity_rear_left,
                    self.update_proximity_rear_right
                    ]

        self.proximity_subscribers = [
            rospy.Subscriber(sensor, Range, callback[i])
            for i, sensor in enumerate(self.proximity_sensors_name)
        ]

        self.position = Point()
        self.orientation = 0
        self.proximity = [0.12] * 7
        self.covariance = None

    def update_pose(self, data):
        self.position = data.pose.pose.position
        self.covariance = data.pose.covariance
        quaternion = data.pose.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        _, _, self.orientation = euler_from_quaternion(explicit_quat)
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

    def update_proximity_rear_left(self, data):
        self.proximity[5] = data.range

    def update_proximity_rear_right(self, data):
        self.proximity[6] = data.range

    def run(self):

        if self.mode == 'FOLLOW8':
            self.controller = Follow8Controller()
        elif self.mode == 'EXPLORER':
            self.controller = ExplorerController()
        else:
            self.controller = WallController(0.025)

        while not rospy.is_shutdown():
            if self.mode == 'FOLLOW8':
                vel = self.controller.run(self.orientation)
            elif self.mode == 'EXPLORER':
                vel = self.controller.run(self.proximity, self.position, self.orientation)
            else:
                vel = self.controller.run(self.proximity, self.position, self.orientation)

            self.velocity_publisher.publish(vel)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        Thymio_controller = Mighty_Thymio()
        Thymio_controller.run()
    except rospy.ROSInterruptException as e:
        pass
