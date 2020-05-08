#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist, Vector3
from lg_msg_defs.msg import State

NODE_NAME = 'wiimote_to_twist'
MAX_AXIS_VAL = 1 / math.sqrt(2.0)
ZOOM_BUTTON_VEL = 8.0
PAN_BUTTON_VEL = 8.0
TILT_BUTTON_VEL = 8.0
TICK_RATE = 65.0  # Hz


def clamp(val, lo, hi):
    return min(max(val, lo), hi)


class WiiMoteToTwist:
    def __init__(self, twist_pub, scale):
        self.twist_pub = twist_pub
        self.scale = scale

    def handle_wiimote_state(self, msg):
        twist = Twist()

        if msg.buttons[0]:
            # 1
            twist.angular.y -= TILT_BUTTON_VEL * self.scale
        if msg.buttons[1]:
            # 2
            twist.angular.y += TILT_BUTTON_VEL * self.scale
        if msg.buttons[2]:
            # plus
            twist.linear.z -= ZOOM_BUTTON_VEL * self.scale
        if msg.buttons[3]:
            # minus
            twist.linear.z += ZOOM_BUTTON_VEL * self.scale
        if msg.buttons[6]:
            # up
            twist.linear.x += PAN_BUTTON_VEL * self.scale
        if msg.buttons[7]:
            # down
            twist.linear.x -= PAN_BUTTON_VEL * self.scale
        if msg.buttons[8]:
            # left
            twist.linear.y += PAN_BUTTON_VEL * self.scale
        if msg.buttons[9]:
            # right
            twist.linear.y -= PAN_BUTTON_VEL * self.scale

        twist.linear.x = clamp(twist.linear.x, -MAX_AXIS_VAL, MAX_AXIS_VAL)
        twist.linear.y = clamp(twist.linear.y, -MAX_AXIS_VAL, MAX_AXIS_VAL)
        twist.linear.z = clamp(twist.linear.z, -MAX_AXIS_VAL, MAX_AXIS_VAL)
        twist.angular.x = clamp(twist.angular.x, -MAX_AXIS_VAL, MAX_AXIS_VAL)
        twist.angular.y = clamp(twist.angular.y, -MAX_AXIS_VAL, MAX_AXIS_VAL)
        twist.angular.z = clamp(twist.angular.z, -MAX_AXIS_VAL, MAX_AXIS_VAL)

        self.twist_pub.publish(twist)


def main():
    rospy.init_node(NODE_NAME)

    scale = float(rospy.get_param('~scale', 0.05))

    twist_pub = rospy.Publisher('/lg_pointer/twist', Twist, queue_size=10)

    wmtt = WiiMoteToTwist(twist_pub, scale)

    rospy.Subscriber('/wiimote/state', State, wmtt.handle_wiimote_state)

    rospy.spin()


if __name__ == '__main__':
    main()
