#!/usr/bin/python3

#import time, tracback
import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped


class MultiPanFix(object):
    """ emit /lg_navlib/twist messages with 'pan x' where 'only zoom' detected """
    def __init__(self, min_zoom=0.0005, min_pan=0.1):
        self.min_zoom = min_zoom
        self.min_pan = min_pan
        self.node = rospy.init_node('zoom_panner')
        self.puber = rospy.Publisher('/spacenav/twist', Twist, queue_size=10)
        self.sub_twist = rospy.Subscriber("/lg_navlib/twist", Twist, self.update_twist)
        self.sub_pose = rospy.Subscriber("/earth/pose", PoseStamped, self.update_pose)
        self.altitude = 0.0
        self.tilt = 0.0
        self.globe_view_altitude = 50000

    def update_pose(self, pose):
        """ update tilt, any other values from pose """
        self.tilt = pose.pose.orientation.x
        self.altitude = pose.pose.position.z
        # time.sleep(0.1)
        # print(f"pose updated {self.tilt}, {pose.pose.orientation.x} and {self.altitude}, {pose.pose.position.z}")

    def update_twist(self, twist):
        """ if twist meets conditions fix, publish """
        # act on zoom input, ignore over an altitude
        if abs(twist.linear.z) > self.min_zoom and self.altitude < self.globe_view_altitude:
            # vVvVv cut this off allowing zoom corrections while other movements
            # and not any(((abs(m) > self.k) for m in (twist.linear.x, twist.linear.y, twist.angular.x, twist.angular.y, twist.angular.z))):
            print(f"tilt: {self.tilt}, altitude: {self.altitude}, sqr alt: {math.sqrt(self.altitude)}, twist in zoom: {twist} ")
            twist.linear.x = -(max(self.tilt, 0.0000001) * twist.linear.z * max(math.sqrt(self.altitude), 0.0000001))
            # reset the rest to close to 0, probably causing some stuttering or interference
            # commented to allow corrections while other moves
            # twist.linear.z, twist.linear.y, twist.angular.x, twist.angular.y, twist.angular.z = 0.0000001, 0.0000001, 0.0000001, 0.0000001, 0.0000001
            # reset zoom only instead
            twist.linear.z = 0.0000001
            print(f"twist out pan x: {twist.linear.x}")
        else:
            # pan boost test fix. -should be deactivated all together, pan works on some kind of click lock
            # boost ys
            while abs(twist.linear.y) < self.min_pan:
                twist.linear.y = (twist.linear.y * twist.linear.y)
            twist.linear.y = (twist.linear.y * self.altitude * self.altitude)
            print(f"twist out pan y: {twist.linear.y}")
            # boost xs
            while abs(twist.linear.x) < self.min_pan:
                twist.linear.x = (twist.linear.x * twist.linear.x)
            twist.linear.x = (twist.linear.y * self.altitude * self.altitude)
            print(f"twist out pan x: {twist.linear.x}")
            # reset the rest to close to 0, probably causing some stuttering or interference
            # commented to allow corrections while other moves
            # twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z = 0.0000001, 0.0000001, 0.0000001, 0.0000001
        # test tilt correction
        # twist.angular.x = twist.angular.x + 0.1
        # print(f"out {twist}")
        self.puber.publish(twist)


if __name__ == '__main__':
    MultiPanFix(min_zoom=0.00000001, min_pan=0.0001)
    rospy.spin()
