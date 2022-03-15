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
        #twist.linear.x = tiltness * down if twist.linear.z != 0 else twist.linear.x * (1 + tiltness)
        #twist.linear.x *= 1 + tiltness
        #twist.linear.y *= 1 + tiltness
        #twist.linear.x = twist.linear.x * (1.0 if twist.linear.z == 0 else 1 + tiltness)
        self.puber.publish(twist)
        """
        # pan for zoom (won't be used if panning before zoom)
        if abs(twist.linear.z) > self.min_zoom and self.altitude < self.globe_altitude and self.tilt >= 1:
            # no tilting/rotating while zooming.
            twist.angular.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0

            # mathit
            #twist.linear.x = (twist.linear.z * self.altitude) / (1080 / (91 - self.tilt))

            # print(f"tilt: {self.tilt}, altitude: {self.altitude}")
            tilt_varying_zoom = (twist.linear.z / 90) * max(((self.tilt + (2 * (self.tilt - 45))) + 15), self.tilt * 0.0001)
            # factor in tilt on altitude multiplier
            tilt_altitude = self.altitude * self.tilt / 90
            twist.linear.x = -tilt_varying_zoom * tilt_altitude
            # print(f'tvz: {tilt_varying_zoom}, ta: {tilt_altitude}, out_pan_x: {twist.linear.x}')
            # test
            #twist.linear.x = -(twist.linear.z / 90) * (self.altitude * self.tilt / 90) * (self.tilt + (self.tilt ** 2 / 90))
            print(f"in_zoom: {twist.linear.z}, out pan x: {twist.linear.x}")

            # zoom less at higher tilts when panning
            if self.tilt < 90 and self.altitude < self.globe_altitude:
                twist.linear.z = 90 * twist.linear.z / (90 - self.tilt)
                print(f"zoom corrected to {twist.linear.z}")

        # correct tilt towards 0 when zooming out past an altitude so earth will be centered on screen
        if twist.linear.z > self.min_zoom and self.altitude > self.globe_altitude and self.tilt > 1:
            twist.angular.y = 0.1   #self.altitude / self.globe_altitude
            print(f"tilt: {self.tilt}, tilt correct: {twist.angular.y}")

        # no zoom over 90 tilt
        if self.tilt >= 90:
            twist.linear.z = -0.00000000001
            print(f'tilt: {self.tilt} so zoom: {twist.linear.z}')

        # publish all changed and unchanged
        print(f"twist out: {twist}")
        self.puber.publish(twist)
        """


if __name__ == '__main__':
    MultiPanFix()
    rospy.spin()
