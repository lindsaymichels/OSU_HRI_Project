#!/usr/bin/env python3

import rospy
import actionlib
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Twist, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseResult
import numpy as np

# Maximum speed for the controller
MAX_SPEED = 0.4
# Proportional gain for the controller
k_p_linear  = 0.4  
# Proportional gain for the angular controller
k_p_angular = 0.4

class DriveToXYNode:

    def __init__(self):
        rospy.init_node('drive_to_xy', anonymous=True)
        
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/quori/base_controller/cmd_vel', Twist, queue_size=10)
        
        self.listener = tf.TransformListener()
        
        self.action_server = actionlib.SimpleActionServer('drive_to_point', MoveBaseAction, self.execute, False)
        self.action_server.start()

    def execute(self, goal):
        # Calculate midpoint between the two points
        points = [
            {"x": -2.0736373240214316, "y": 0.8298911639062807, "z": 4.103},
            {"x": -1.0883985075612308, "y": 0.6883670810084711, "z": 3.436}
        ]
        
        midpoint_x = np.mean([point['x'] for point in points])
        midpoint_y = np.mean([point['y'] for point in points])

        rospy.loginfo(f"Midpoint: ({midpoint_x}, {midpoint_y})")
        
        # Set the midpoint as the target
        target_x = midpoint_x
        target_y = midpoint_y

        # Publish a marker at the midpoint location
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = target_x
        marker.pose.position.y = target_y
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        rate = rospy.Rate(10.0)
        feedback = MoveBaseFeedback()
        twist = Twist()

        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Publishing marker")
                self.marker_pub.publish(marker)

                (trans, rot) = self.listener.lookupTransform('/map', 'ramsis/wheel_axle', rospy.Time(0))
                current_x, current_y = trans[0], trans[1]
                
                # Calculate the difference between the current position and the target position
                error_x = target_x - current_x
                error_y = target_y - current_y
                distance = np.hypot(error_x, error_y)

                if distance < 0.1:
                    # Close enough to the goal
                    rospy.loginfo("Reached the goal!")
                    twist.linear.x = 0
                    twist.angular.z = 0
                    self.cmd_vel_pub.publish(twist)
                    result = MoveBaseResult()
                    self.action_server.set_succeeded(result)
                    break

                # Calculate the desired heading
                desired_yaw = np.arctan2(error_y, error_x)

                # Get current yaw from quaternion
                current_yaw = tf.transformations.euler_from_quaternion(rot)[2]

                # Calculate the yaw error
                error_yaw = desired_yaw - current_yaw

                # Normalize the yaw error to the range [-pi, pi]
                error_yaw = np.arctan2(np.sin(error_yaw), np.cos(error_yaw))

                # Proportional controller for angular velocity
                twist.angular.z = k_p_angular * error_yaw
                if twist.angular.z > MAX_SPEED:
                    twist.angular.z = MAX_SPEED

                if abs(error_yaw) < 0.1:
                    # If facing the target, move forward
                    twist.linear.x = k_p_linear * distance
                    if twist.linear.x > MAX_SPEED:
                        twist.linear.x = MAX_SPEED
                else:
                    # If not facing the target, do not move forward
                    twist.linear.x = 0
                
                # Publish the velocity command
                rospy.loginfo(f"Moving to: ({current_x}, {current_y})")
                self.cmd_vel_pub.publish(twist)

                # Update and publish feedback
                feedback.base_position.pose.position.x = current_x
                feedback.base_position.pose.position.y = current_y
                self.action_server.publish_feedback(feedback)

                rate.sleep()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


if __name__ == '__main__':
    try:
        DriveToXYNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
