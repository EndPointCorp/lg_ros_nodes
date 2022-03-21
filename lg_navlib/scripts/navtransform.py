#!/usr/bin/python3

import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped


class NavTransform(object):
    """ emit /lg_navlib/twist messages with 'pan x' where 'zoom' detected """
    def __init__(self):
        self.node = rospy.init_node('navtransform')
        self.puber = rospy.Publisher('/navtransform/twist', Twist, queue_size=10)
        self.sub_twist = rospy.Subscriber("/lg_navlib/twist", Twist, self.transform_twist)
        self.sub_pose = rospy.Subscriber("/earth/pose", PoseStamped, self.update_pose)
        self.tilt = 0.0
        self.altitude = 0.0
        self.lastrotate = 0.0

    def update_pose(self, pose):
        """ update tilt, any other values from pose """
        self.tilt = pose.pose.orientation.x
        self.altitude = pose.pose.position.z

    def transform_twist(self, twist):
        """ fix twist, publish """
#       # convert zoom into panned-zoom
        down = min(max(twist.linear.z, -1), 1) * -1
        tiltness = min(1.0, max(0.0, self.tilt / 90))
        if twist.linear.z != 0:
            twist.linear.x = twist.linear.x * (1 - tiltness) + down * tiltness
            twist.linear.z *= 1.0 - tiltness

#       # correct pan
        twist.linear.x /= 2

#       # limit tilt-up to 65-60
        if twist.angular.y < 0:
            if self.tilt > 70:
                twist.angular.y = 0.05
            elif self.tilt > 65:
                twist.angular.y = 0.0
            elif self.tilt >= 55:
                twist.angular.y /= 3

#       # add pan when tilting
        if twist.angular.y != 0 and self.tilt > 0:
            twist.angular.y /= 4
            twist.linear.x = 1.25 * twist.angular.y * self.tilt / 90

#       # correct tilt when zooming-in close to the ground
        if twist.linear.z < 0 and self.altitude < 10000 and self.tilt < 50:
            twist.angular.y = twist.linear.z / 2

        if abs(twist.angular.z) > 0:
#            # ignore random big values that come in :P
            if not abs(self.lastrotate) * 2 > abs(twist.angular.z) > abs(self.lastrotate) / 2:
                self.lastrotate = twist.angular.z
                twist.angular.z = 0.0
            else:
                self.lastrotate = twist.angular.z
                twist.angular.z /= 2
                twist.linear.z, twist.angular.x, twist.angular.y = 0.0, 0.0, 0.0
#               # correct position while rotating to keep subject on screen
                twist.linear.y = -1 * twist.angular.z * self.tilt / 90
                twist.linear.x = twist.linear.y / 4

#       # correct tilt when zooming-out to keep earth on screen
        if twist.linear.z > 0 and self.altitude > 1000000 and self.tilt > 0:
            twist.angular.y = 0.4

        self.puber.publish(twist)


if __name__ == '__main__':
    NavTransform()
    rospy.spin()
