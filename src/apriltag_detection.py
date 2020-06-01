#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray

class CameraPosition():
    def __init__(self):
        rospy.wait_for_message('/tag_detections', AprilTagDetectionArray)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.get_tags)
        rospy.loginfo("Publishing april odometry on /odom_april")

    def get_tags(self, msg):
        if len(msg.detections) > 0:
            pos_x = msg.detections[0].pose.pose.pose.position.x
            print(pos_x)

if __name__ == "__main__":
    rospy.loginfo("Initializing node My-detection... ")
    rospy.init_node("mydectetion")
    CameraPosition()
    print('Running my detection')
    rospy.spin()