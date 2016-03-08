#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from lg_common.helpers import write_log_to_file

GUTTER_VAL = 0.01

class MockPublisher(object):
    def publish(self, *args, **kwargs):
        pass

def MockFunc(*args, **kwargs):
    pass

class SpacenavWrapper(object):
    def __init__(self, twist=MockPublisher(), joy=MockPublisher(),
                 rezero=MockFunc, gutter_val=GUTTER_VAL,
                 translate_twist=MockFunc):
        self.twist = twist
        self.joy = joy
        self.rezero = rezero
        self.gutter_val = gutter_val
        self.translate_twist = translate_twist

        self.msg_buffer = []
        self.buffer_size = 20

    def handle_twist(self, msg):
        """
        Gets twist message, and publishes it or a cleaned up version
        of it
        """
        if self._needs_rezero(msg):
            write_log_to_file('rezeroing...')
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
        return abs(twist_msg.linear.x) < self.gutter_val and \
            abs(twist_msg.linear.y) < self.gutter_val and \
            abs(twist_msg.linear.z) < self.gutter_val and \
            abs(twist_msg.angular.x) < self.gutter_val and \
            abs(twist_msg.angular.y) < self.gutter_val and \
            abs(twist_msg.angular.z) < self.gutter_val

    def _is_twist_equal(self, twist1, twist2):
        return twist1.linear.x == twist2.linear.x and \
            twist1.linear.y == twist2.linear.y and \
            twist1.linear.z == twist2.linear.z and \
            twist1.angular.x == twist2.angular.x and \
            twist1.angular.y == twist2.angular.y and \
            twist1.angular.z == twist2.angular.z

    def _needs_rezero(self, twist_msg):
        """
        Checks the last $buffer_size messages, if they're all the same,
        then returns True
        """
        if self.rezero == None:
            return False
        # append the msg, and save only the last $buffer_size
        self.msg_buffer.append(twist_msg)
        self.msg_buffer = self.msg_buffer[self.buffer_size * -1:]

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
        write_log_to_file('rezeroing with buffer of: (%s)' % self.msg_buffer)
        return True
