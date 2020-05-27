#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


GUTTER_VAL = 0.01


class MockPublisher(object):
    def publish(self, *args, **kwargs):
        pass


def MockFunc(*args, **kwargs):
    pass

# every 10ms:
# check if axis is within threshold of previous sample
# if within threshold, increment counter
# else, store sample
# if counter > max, rezero and reset

# THRESHOLD = 3 * FULL_SCALE
# SECONDS_BEFORE_REZERO = 30.0
# RATE = 100.0
# PERIOD = 1.0 / RATE
# MAX_COUNTS = RATE * SECONDS_BEFORE_REZERO
#
#
# def reset():
#   counter = 0
#   old_sample = Twist()
#   new_sample = Twist()
# def on_spacenav_msg(msg):
#   new_sample = msg
# def magnitude(vector):
#   return vector.x + vector.y + vector.z
# def diff_twist(t1, t2):
#   return (
#     magnitude(t1.linear) - magnitude(t2.linear),
#     magnitude(t1.angular) - magnitude(t2.angular),
#   )
# def on_timer(t):
#   if twist_equal(new_sample, Twist()):
#     return
#   diff_linear, diff_angular = diff_twist(new_sample, old_sample)
#   if abs(diff_linear) < THRESHOLD and abs(diff_angular) < THRESHOLD:
#     counter += 1
#   else:
#     old_sample = new_sample
#     counter = 0
#   if counter > MAX_COUNTS:
#     rezero()
#     reset()


class SpacenavRezeroer(object):
    def __init__(self, threshold, seconds_before_rezero, rate=100, rezero=MockFunc):
        self.threshold = threshold
        self.seconds_before_rezero = seconds_before_rezero
        self.rate = rate
        self.max_counts = self.rate * self.seconds_before_rezero
        self.rezero = rezero

        # this also initializes our variables
        self.reset()

    def reset(self):
        self.counter = 0
        self.new_sample = Twist()
        self.old_sample = Twist()

    def _on_timer(self):

        if self._is_twist_equal(self.new_sample, Twist(), epsilon=0):
            # we want to ignore zero messages, but not reset the counters
            # because it is possible a spacenav is spinning around the
            # zero value and hitting it
            return

        diff_linear, diff_angular = self.diff_twist(self.new_sample, self.old_sample)
        if diff_linear < self.threshold and diff_angular < self.threshold:
            self.counter += 1
        else:
            self.old_sample = self.new_sample
            self.counter = 0
        if self.counter >= self.max_counts:
            rospy.loginfo('rezeroing after counter reached %s, old_sample:\n%s\nnew_sample:\n%s' % (
                self.counter, self.old_sample, self.new_sample
            ))
            self.rezero()
            self.reset()

    def diff_twist(self, t1, t2):
        """
        Diff the both magnitudes of each twist and
        return a tuple of the absolute value of those differences
        """
        def magnitude(vector):
            return vector.x + vector.y + vector.z
        return (
            abs(magnitude(t1.linear) - magnitude(t2.linear)),
            abs(magnitude(t1.angular) - magnitude(t2.angular)),
        )

    def on_spacenav(self, msg):
        self.new_sample = msg

    def on_timer(self, *args, **kwargs):
        """
        Wrapper for timer method, we need to catch all exceptions inside of
        our timer, or it will die and the node will never know
        """
        try:
            self._on_timer()
        except Exception as e:
            rospy.logerr("There was an exception thrown inside the timer:\n%s" % e)

    def _is_twist_equal(self, twist1, twist2, epsilon=0.005):
        return abs(twist1.linear.x - twist2.linear.x) <= epsilon and \
            abs(twist1.linear.y - twist2.linear.y) <= epsilon and \
            abs(twist1.linear.z - twist2.linear.z) <= epsilon and \
            abs(twist1.angular.x - twist2.angular.x) <= epsilon and \
            abs(twist1.angular.y - twist2.angular.y) <= epsilon and \
            abs(twist1.angular.z - twist2.angular.z) <= epsilon


class SpacenavWrapper(object):
    def __init__(self, twist=MockPublisher(), joy=MockPublisher(),
                 gutter_val=GUTTER_VAL, translate_twist=MockFunc):
        self.twist = twist
        self.joy = joy
        self.gutter_val = gutter_val
        self.translate_twist = translate_twist
        self.suppressed = False

    def suppress(self, suppressed):
        self.suppressed = suppressed

    def handle_twist(self, msg):
        """
        Gets twist message, and publishes it or a cleaned up version
        of it
        """
        if self.suppressed:
            return
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
