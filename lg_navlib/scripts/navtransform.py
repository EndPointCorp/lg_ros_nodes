#!/usr/bin/python3

import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped


class NavTransform(object):
    """ correct /lg_navlib/twist messages into natural movement, emit on /navtransform/twist """
    def __init__(self):
        self.node = rospy.init_node('navtransform')
        self.puber = rospy.Publisher('/navtransform/twist', Twist, queue_size=10)
        self.sub_twist = rospy.Subscriber("/lg_navlib/twist", Twist, self.transform_twist)
        self.sub_pose = rospy.Subscriber("/earth/pose", PoseStamped, self.update_pose)
        self.tilt = 0.0
        self.altitude = 0.0

    def update_pose(self, pose):
        """ update tilt and altitude from /earth/pose """
        self.tilt = pose.pose.orientation.x
        self.altitude = pose.pose.position.z

    def transform_twist(self, twist):
        """ fix twist, publish """
#       # increase zoom speed with tilt
        twist.linear.z = (twist.linear.z + (twist.linear.z * max(self.tilt, 0.01) / 90))

#       # convert zoom into panned-zoom
        down = min(max(twist.linear.z, -1), 1) * -1
        tiltness = min(1.0, max(0.0, self.tilt / 90))
        if twist.linear.z != 0:
            twist.linear.x = twist.linear.x * (1 - tiltness) + down * tiltness
            twist.linear.z *= 1.0 - tiltness

#       # limit tilt-up to 85-90
        if twist.angular.y < 0:
            if self.tilt > 90:
                twist.angular.y = 0.05
            elif self.tilt > 85:
                twist.angular.y = 0.0
#           # slow down tilt-up before topping off
            elif self.tilt >= 75:
                twist.angular.y *= 0.75

#       # add pan when tilting
        if twist.angular.y != 0 and self.tilt > 0 and self.altitude < 500000:
            twist.linear.x = twist.angular.y * self.tilt / 90

#       # correct tilt when zooming-in close to the ground
        if twist.linear.z < 0 and self.altitude < 3000 and self.tilt < 70:
            twist.angular.y = 1.5 * twist.linear.z

#       # correct position while rotating to keep subject on screen
        if abs(twist.angular.z) > 0:
            twist.angular.z *= 0.75
            twist.linear.y = -1 * twist.angular.z * self.tilt / 90
            twist.linear.x = twist.linear.y / 4

#       # correct tilt when zooming-out to keep earth on screen
        if twist.linear.z > 0 and self.altitude > 1000000 and self.tilt > 0:
            twist.angular.y = 0.4

        self.puber.publish(twist)


if __name__ == '__main__':
    NavTransform()
    rospy.spin()
