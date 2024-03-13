#!/usr/bin/python3

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
#        self.altitude = pose.pose.position.z

    def transform_twist(self, twist):
        """ fix twist, publish """
#       increase zoom speed with tilt
        twist.linear.z = (twist.linear.z + (twist.linear.z * max(self.tilt, 0.01) / 90))

        self.puber.publish(twist)


if __name__ == '__main__':
    NavTransform()
    rospy.spin()
