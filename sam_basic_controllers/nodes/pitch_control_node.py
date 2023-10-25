#!/usr/bin/env python3
"""
Node for depth control using the VBS
"""

from __future__ import division, print_function

import rospy

from std_msgs.msg import Float64
from sam_msgs.msg import PercentStamped



class PitchControlNode(object):
    def __init__(self):

        # Read external parameters
        self.loop_frequency = rospy.get_param("~loop_frequency", 10.0)
        self.Ki = rospy.get_param("~Ki", 0.0)
        self.Kp = rospy.get_param("~Kp", 0.0)
        self.Kd = rospy.get_param("~Kd", 0.0)
        self.Kaw = rospy.get_param("~Kaw", 0.0)
        self.lcg_neutral = rospy.get_param("~lcg_neutral", 0.0)

        # Initialize variables
        self.u = self.lcg_neutral
        self.u_limit = self.lcg_neutral
        # TODO: Check the sign of the depth message and adjust the controller accordingly
        self.current_pitch = 0.0
        self.pitch_setpoint = 0.0
        self.error = 0.0
        self.error_prev = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.anti_windup_integral = 0.0

        # Topics
        # TODO: Get the right topic names
        lcg_topic = rospy.get_param("pitch_topic", "/sam/core/vbs_cmd")
        current_pitch_topic = rospy.get_param("~current_pitch_topic", "/sam/core/current_depth")
        pitch_setpoint_topic = rospy.get_param("~pitch_setpoint_topic", "/sam/core/depth_setpoint")

        # Subscribers
        # TODO: Check the message types and update the callbacks
        rospy.Subscriber(current_pitch_topic, Float64, self.current_pitch_callback)
        rospy.Subscriber(pitch_setpoint_topic, Float64, self.pitch_setpoint_callback)

        # Publishers
        self.lcg_pub = rospy.Publisher(lcg_topic, PercentStamped, queue_size=1)

        rate = rospy.Rate(self.loop_frequency)

        # Run
        while not rospy.is_shutdown():
            self.compute_control_action()
            self.limit_control_action()
            self.publish_control_action()

            rate.sleep()

    #region Callbacks
    # TODO: Upate the callbacks with the right message types
    def current_pitch_callback(self, msg):
        """
        Callback for the current pitch
        """
        self.current_pitch = msg.data

    def pitch_setpoint_callback(self, msg):
        """
        Callback for the pitch setpoint
        """
        self.pitch_setpoint = msg.data
    #endregion

    #region Functions
    def compute_control_action(self):
        """
        Compute the control action
        """
        self.error_prev = self.error
        self.error = self.pitch_setpoint - self.current_pitch

        self.compute_anti_windup()

        self.integral += self.error / self.loop_frequency
        self.derivative = (self.error - self.error_prev) * self.loop_frequency

        self.u = self.Kp * self.error \
                + self.Ki * self.integral \
                + self.Kd * self.derivative \
                + self.Kaw * self.anti_windup_integral \
                + self.lcg_neutral

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
        self.lcg_pub.publish(self.u_limit)
    #endregion


if __name__ == "__main__":
    rospy.init_node('PitchControlNode')
    try:
        PitchControlNode()
    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch pitch control node")
