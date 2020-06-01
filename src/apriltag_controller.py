#!/usr/bin/env python

import rospy
import movement_utils as mv
from pid import PID
from geometry_msgs.msg import Twist, Pose, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from math import pi, cos, sin
from apriltag_ros.msg import AprilTagDetectionArray


class ModeType:
    FOLLOW_TAG = 1
    GO_HOME = 2


class AprilTagController:
    def __init__(self, proximity_threshold=0.035, wall_safety_distance=2., mode=ModeType.FOLLOW_TAG, debug=False):
        rospy.logdebug('Waiting for tag detection')
        rospy.wait_for_message('/tag_detections', AprilTagDetectionArray)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.get_tags)
        rospy.logdebug("Publishing april odometry on /odom_april")

        self.proximity_min_range = 0.010
        self.proximity_max_range = 0.12
        # Distance between the rear proximity sensors and the reference of the robot
        self.rear_proximity_offset = 0.03
        # Extra distance from wall to cover once the proximity rear sensor stop detecting the wall
        self.distance = wall_safety_distance - self.proximity_max_range - self.rear_proximity_offset

        # --- STATE VARIABLES ---
        self.TAG_DETECTED = False
        self.LOST_DETECTION = False
        self.WALL_REACHED = False
        self.PERPENDICULAR = False
        self.ROTATING = False
        self.ROTATED = False
        self.MAX_RANGE = False
        self.DONE = False
        self.flat_surface = False

        self.pose_to_marker = None

        self.proximity_threshold = proximity_threshold
        self.motion_controller = mv.ToTargetPController(linear_speed=0.20, orientation_speed=2.5)
        self.debug = debug
        self.final_target = Pose()

        self.angular_vel_pid = PID(Kp=20., Ki=0., Kd=0.)
        self.velocity = Twist()

    def get_tags(self, msg):
        if len(msg.detections) > 0:
            tagDetection = msg.detections[0]
            self.pose_to_marker = tagDetection.pose.pose.pose
            self.TAG_DETECTED = True
            self.LOST_DETECTION = False
            print('Found tag %s %s' % (tagDetection.id[0], self.pose_to_marker))
        else:
            if self.TAG_DETECTED:
                self.LOST_DETECTION = True

            self.TAG_DETECTED = False

    def move_closer(self, proximity, position, orientation):
        # Move ahead until the proximity sensors detect an obstacle,
        # then get closer than proximity threshold meters

        if proximity[1] > 0.11 and proximity[2] > 0.11 and proximity[3] > 0.11:
            self.velocity.linear.x = .14
        elif proximity[1] > self.proximity_threshold and proximity[2] > self.proximity_threshold \
                and proximity[3] > self.proximity_threshold:
            self.velocity.linear.x = 0.033
        else:
            self.velocity.linear.x = 0.
            self.WALL_REACHED = True

            self.flat_surface = True
            for i in range(1, 4):
                if proximity[i] > 0.119:
                    self.flat_surface = False
            print("Obstacle reached. Turning...")

    """
    If the obstacle is sufficiently wide the proximity sensor are used to rotate the robot to be orthogonal 
    to the obstacle, otherwise no alignment is done (i.e. the robots simply turns by 180 deg with respect
    to the initial direction.
    """

    def align_perpendicular(self, proximity, position, orientation):
        # Use the frontal proximity sensor to rotate the robot until its is perpendicular to the wall
        max_orientation_speed = 0.75

        if self.flat_surface:
            angular_vel = self.angular_vel_pid.step(proximity[3] - proximity[1], dt=0.1)
            module = min(abs(angular_vel), max_orientation_speed)
            angular_vel *= module / abs(angular_vel)

            self.velocity.linear.x = 0.
            self.velocity.angular.z = angular_vel
        else:
            angular_vel = 0.

        if abs(angular_vel) < 0.01:
            self.PERPENDICULAR = True
            self.target_orientation = (orientation + pi) % (2 * pi)

    def turn_180(self, proximity, position, orientation):
        done, vel = self.motion_controller.move(position, orientation,
                                                position, target_orientation=self.target_orientation,
                                                max_orientation_speed=.75
                                                )
        self.velocity.linear.x = vel.linear.x
        self.velocity.angular.z = vel.angular.z

        if done:
            self.ROTATED = True
            print("Done.")

    def move_max_range(self, proximity, position, orientation):
        # Move the robot away until the rear proximity sensors reach max range

        self.velocity.angular.z = 0.

        if proximity[5] < 0.1199:
            self.velocity.linear.x = 0.1
        else:
            self.velocity.linear.x = 0.

            self.final_target = Point()

            # If self.distance < 0, then the robot is already far enough from the wall
            self.final_target.y = position.y + sin(orientation) * (self.distance if self.distance > 0 else 0.)
            self.final_target.x = position.x + cos(orientation) * (self.distance if self.distance > 0 else 0.)
            self.MAX_RANGE = True

    def move_away(self, proximity, position, orientation):
        # Move away from obstacle until safety distance is reached
        done, vel = self.motion_controller.move(position, orientation,
                                                self.final_target, max_linear_speed=0.2
                                                )
        self.velocity.linear.x = vel.linear.x
        self.velocity.angular.z = vel.angular.z

        if self.debug:
            print(self.velocity)
            print("{0} --> {1}".format(position.x, self.final_target.x))
        if done:
            # print("Done. Final position: ({0},{1})".format(position.x,position.y))
            self.DONE = True

    def move_closer_to_tag(self, proximity, position, orientation):
        rospy.logdebug('Position %s and Orientation %s' % (position, orientation))
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        rospy.logdebug('Stop robot')
        self.DONE = True
        return
        self.final_target.x = position.x + cos(orientation)
        self.final_target.y = position.y + sin(orientation)

        done, vel = self.motion_controller.move(position, orientation, self.final_target, max_linear_speed=0.2)
        print('Move closer to tag')
        if done:
            self.DONE = True

    def reset(self):
        self.WALL_REACHED = False
        self.PERPENDICULAR = False
        self.ROTATING = False
        self.ROTATED = False
        self.MAX_RANGE = False
        self.DONE = False

    @staticmethod
    def is_obstacle_present(proximity):
        return False if proximity[1] > 0.11 and proximity[2] > 0.11 and proximity[3] > 0.11 else True

    def run(self, proximity, position, orientation):

        rospy.logdebug('Orientation %s' % orientation)
        if not self.WALL_REACHED and not self.TAG_DETECTED:
            rospy.logdebug('Not Wall reached nor Detected')
            self.move_closer(proximity, position, orientation)

        if self.WALL_REACHED and not self.PERPENDICULAR:
            self.align_perpendicular(proximity, position, orientation)

        if not self.ROTATED and self.PERPENDICULAR:
            self.turn_180(proximity, position, orientation)

        if not self.MAX_RANGE and self.ROTATED:
            self.move_max_range(proximity, position, orientation)

        if not self.DONE and self.MAX_RANGE:
            self.move_away(proximity, position, orientation)

        if self.TAG_DETECTED and not self.LOST_DETECTION:
            self.move_closer_to_tag(proximity, self.pose_to_marker.position, self.pose_to_marker.orientation)

        if self.LOST_DETECTION:
            print('Rotate 360 until find other tags, revert orientation if not found any')

        if self.DONE:
            self.velocity.linear.x = 0.
            self.velocity.angular.z = 0.

        return self.velocity, True
