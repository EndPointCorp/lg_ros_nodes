#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

GUTTER_VAL = 0.01


class MockPublisher(object):
    def publish(self, *args, **kwargs):
        pass


def MockFunc(*args, **kwargs):
    pass


class SpacenavWrapper(object):
    def __init__(self, twist=MockPublisher(), joy=MockPublisher(),
                 rezero=MockFunc, gutter_val=GUTTER_VAL,
                 translate_twist=MockFunc, full_scale=0,
                 buffer_size=200):
        self.twist = twist
        self.joy = joy
        self.rezero = rezero
        self.gutter_val = gutter_val
        self.translate_twist = translate_twist
        # 350 is the max on full scale (which is either 0 or 1), divide by 2 to get
        # the mid range, and add 0.5 in case there was no full scale
        self.mid_value = full_scale * 350 / 2 + 0.5

        self.msg_buffer = []
        self.buffer_size = buffer_size

    def handle_twist(self, msg):
        """
        Gets twist message, and publishes it or a cleaned up version
        of it
        """
        if self._needs_rezero(msg):
            rospy.loginfo('needs rezero...')
            self.rezero()
        if self._is_in_gutter(msg):
            msg = Twist()
        self.translate_twist(msg)
        self.twist.publish(msg)

    def handle_joy(self, msg):
        pass

    def _is_in_gutter(self, twist_msg):
        """
        Returns boolean, whether or not the message is in the gutter
        """
        if self.gutter_val <= 0:
            return False
        return abs(twist_msg.linear.x) < self.gutter_val and \
            abs(twist_msg.linear.y) < self.gutter_val and \
            abs(twist_msg.linear.z) < self.gutter_val and \
            abs(twist_msg.angular.x) < self.gutter_val and \
            abs(twist_msg.angular.y) < self.gutter_val and \
            abs(twist_msg.angular.z) < self.gutter_val

    def _is_twist_equal(self, twist1, twist2, epsilon=0.005):
        return abs(twist1.linear.x - twist2.linear.x) < epsilon and \
            abs(twist1.linear.y - twist2.linear.y) < epsilon and \
            abs(twist1.linear.z - twist2.linear.z) < epsilon and \
            abs(twist1.angular.x - twist2.angular.x) < epsilon and \
            abs(twist1.angular.y - twist2.angular.y) < epsilon and \
            abs(twist1.angular.z - twist2.angular.z) < epsilon

    def _is_twist_near_max(self, twist):
        """
        Checks if the twist message has any fields above 50 percent
        of the max value and returns true if this is true.

        This is needed because it is possible to push the spacenav
        very hard in any direction and get static values for each
        message > self.buffer_size times in a row. This would otherwise
        trigger a relaunch.
        """
        if abs(twist.linear.x) > self.mid_value or \
                abs(twist.linear.y) > self.mid_value or \
                abs(twist.linear.z) > self.mid_value or \
                abs(twist.angular.x) > self.mid_value or \
                abs(twist.angular.y) > self.mid_value or \
                abs(twist.angular.z) > self.mid_value:
            return True
        return False

    def _needs_rezero(self, twist_msg):
        """
        Checks the last $buffer_size messages, if they're all the same,
        then returns True
        """
        if self.rezero is None:
            return False
        # append the msg, and save only the last $buffer_size
        self.msg_buffer.append(twist_msg)
        self.msg_buffer = self.msg_buffer[self.buffer_size * -1:]

        if self._is_twist_near_max(twist_msg):
            return False

        if len(self.msg_buffer) < self.buffer_size:
            return False
        # if the first message is blank, there's no need to check
        # for a re-zero, since it's already at zero
        if self._is_twist_equal(self.msg_buffer[0], Twist()):
            return False

        # compare each message to the first, if we find one that doesn't
        # match, then we don't need a re-zero
        for twist in self.msg_buffer[1:]:
            if not self._is_twist_equal(twist, self.msg_buffer[0]):
                return False
        return True
