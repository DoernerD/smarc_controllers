#!/usr/bin/env python3
"""
Node for depth control using the VBS
"""

from __future__ import division, print_function

import rospy

from std_msgs.msg import Float64
from sam_msgs.msg import PercentStamped



class DepthControlNode(object):
    def __init__(self):

        # Read external parameters
        self.loop_frequency = rospy.get_param("~loop_frequency", 10.0)
        self.Ki = rospy.get_param("~Ki", 0.0)
        self.Kp = rospy.get_param("~Kp", 0.0)
        self.Kd = rospy.get_param("~Kd", 0.0)
        self.Kaw = rospy.get_param("~Kaw", 0.0)
        self.vbs_neutral = rospy.get_param("~vbs_neutral", 0.0)

        # Initialize variables
        self.u = self.vbs_neutral
        self.u_limit = self.vbs_neutral
        # TODO: Check the sign of the depth message and adjust the controller accordingly
        self.current_depth = 0.0    
        self.depth_setpoint = 0.0
        self.error = 0.0
        self.error_prev = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.anti_windup_integral = 0.0

        # Topics
        # TODO: Get the right topic names
        vbs_topic = rospy.get_param("~vbs_topic", "/sam/core/vbs_cmd")
        current_depth_topic = rospy.get_param("~current_depth_topic", "/sam/core/current_depth")
        depth_setpoint_topic = rospy.get_param("~depth_setpoint_topic", "/sam/core/depth_setpoint")

        # Subscribers
        # TODO: Check the message types and update the callbacks
        rospy.Subscriber(current_depth_topic, Float64, self.current_depth_callback)
        rospy.Subscriber(depth_setpoint_topic, Float64, self.depth_setpoint_callback)

        # Publishers
        self.vbs_pub = rospy.Publisher(vbs_topic, PercentStamped, queue_size=1)

        rate = rospy.Rate(self.loop_frequency)

        # Run
        while not rospy.is_shutdown():
            self.compute_control_action()
            self.limit_control_action()
            self.publish_control_action()

            rate.sleep()

    #region Callbacks
    # TODO: Upate the callbacks with the right message types
    def current_depth_callback(self, msg):
        """
        Callback for the current depth
        """
        self.current_depth = msg.data

    def depth_setpoint_callback(self, msg):
        """
        Callback for the depth setpoint
        """
        self.depth_setpoint = msg.data
    #endregion

    #region Functions
    def compute_control_action(self):
        """
        Compute the control action
        """
        self.error_prev = self.error
        self.error = self.depth_setpoint - self.current_depth

        self.compute_anti_windup()

        self.integral += self.error / self.loop_frequency
        self.derivative = (self.error - self.error_prev) * self.loop_frequency

        self.u = self.Kp * self.error \
                + self.Ki * self.integral \
                + self.Kd * self.derivative \
                + self.Kaw * self.anti_windup_integral \
                + self.vbs_neutral

    def compute_anti_windup(self):
        """
        Compute the anti windup
        """
        if self.Kaw != 0:
            self.anti_windup_integral += (self.u_limit - self.u) * 1/self.loop_frequency

    def limit_control_action(self):
        """
        Limit the control action
        """
        if self.u > 100:
            self.u_limit = 100
        elif self.u < 0:
            self.u_limit = 0
        else:
            self.u_limit = self.u

    def publish_control_action(self):
        """
        Publish the control action
        """
        self.vbs_pub.publish(self.u_limit)
    #endregion


if __name__ == "__main__":
    rospy.init_node('DepthControlNode')
    try:
        DepthControlNode()
    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch depth control node")
