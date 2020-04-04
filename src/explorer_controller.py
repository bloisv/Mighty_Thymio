import rospy
import movement_utils as mv
from wall_controller import WallController
from pid import PID
from geometry_msgs.msg import Twist, Pose, Point
from math import pi
import random


class ExplorerController:

    def __init__(self):
        self.obstacle_controller = WallController(proximity_threshold=0.06, wall_safety_distance=.2, debug=False)
        self.motion_controller = mv.ToTargetPController(linear_speed=0.20, orientation_speed=2.5)

        self.INIT = True
        self.EXPLORING = False
        self.OBSTACLE_DETECTED = False
        self.OBSTACLE_AVOIDED = False

        self.velocity = Twist()

        rand_init_dir = random.randint(0, 7)
        self.init_orientation = pi / 4 * rand_init_dir

    def explore(self, proximity, position, orientation):
        self.velocity.angular.z = 0.

        if self.obstacle_controller.is_obstacle_present(proximity):
            self.EXPLORING = False
            self.OBSTACLE_DETECTED = True

            self.velocity.linear.x = 0.
            return

        self.velocity.linear.x = 0.10

    def run(self, proximity, position, orientation):

        if self.INIT:
            done, vel = self.motion_controller.move(position, orientation,
                                                    position, target_orientation=self.init_orientation,
                                                    max_orientation_speed=.75
                                                    )
            self.velocity.linear.x = vel.linear.x
            self.velocity.angular.z = vel.angular.z

            if done:
                self.INIT = False
                self.EXPLORING = True

        if self.EXPLORING:
            self.explore(proximity, position, orientation)

        if self.OBSTACLE_DETECTED:
            vel = self.obstacle_controller.run(proximity, position, orientation)
            self.velocity.linear.x = vel.linear.x
            self.velocity.angular.z = vel.angular.z

            if self.obstacle_controller.DONE:
                self.OBSTACLE_AVOIDED = True
                self.OBSTACLE_DETECTED = False
                direction = 1 if random.random() < 0.5 else -1
                self.new_orientation = mv.to_positive_angle(orientation + direction * pi / 2.)

        if self.OBSTACLE_AVOIDED:
            # Select new direction randomly and start exploring again

            done, vel = self.motion_controller.move(position, orientation,
                                                    position, target_orientation=self.new_orientation,
                                                    max_orientation_speed=.75
                                                    )
            self.velocity.linear.x = vel.linear.x
            self.velocity.angular.z = vel.angular.z

            if done:
                self.EXPLORING = True
                self.OBSTACLE_DETECTED = False
                self.OBSTACLE_AVOIDED = False

        return self.velocity
