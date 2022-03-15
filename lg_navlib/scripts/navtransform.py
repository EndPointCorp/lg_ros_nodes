#!/usr/bin/python3

# import time, tracback
import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped


class MultiPanFix(object):
    """ emit /lg_navlib/twist messages with 'pan x' where 'zoom' detected """
    def __init__(self):
        self.node = rospy.init_node('zoom_panner')
        self.puber = rospy.Publisher('/navtransform/twist', Twist, queue_size=10)
        self.sub_twist = rospy.Subscriber("/lg_navlib/twist", Twist, self.update_twist)
        self.sub_pose = rospy.Subscriber("/earth/pose", PoseStamped, self.update_pose)
        self.tilt = 0

    def update_pose(self, pose):
        """ update tilt, any other values from pose """
        # if pose.pose.orientation.x != 0.0:
        self.tilt = pose.pose.orientation.x

    def update_twist(self, twist):
        """ fix twist, publish """

        down = min(max(twist.linear.z, -1), 1) * -1
        tiltness = min(1.0, max(0.0, self.tilt / 90))
        print(f"tilt: {self.tilt} tiltness: {tiltness}")

        if twist.linear.z != 0:
            twist.linear.x = twist.linear.x * (1 - tiltness) + down * tiltness
            twist.linear.z *= 1.0 - tiltness
        self.puber.publish(twist)


if __name__ == '__main__':
    MultiPanFix()
    rospy.spin()
