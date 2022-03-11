#!/usr/bin/python3

# import time, tracback
import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped


class MultiPanFix(object):
    """ emit /lg_navlib/twist messages with 'pan x' where 'zoom' detected """
    def __init__(self, min_zoom=0.0005, min_pan=0.1):
        self.min_zoom = min_zoom
        self.min_pan = min_pan
        self.node = rospy.init_node('zoom_panner')
        self.puber = rospy.Publisher('/spacenav/twist', Twist, queue_size=10)
        self.sub_twist = rospy.Subscriber("/lg_navlib/twist", Twist, self.update_twist)
        self.sub_pose = rospy.Subscriber("/earth/pose", PoseStamped, self.update_pose)
        self.altitude = 0.0
        self.tilt = 0.0
        self.globe_view_altitude = 30000

    def update_pose(self, pose):
        """ update tilt, any other values from pose """
        if pose.pose.orientation != 0.0:
            self.tilt = pose.pose.orientation.x
        if pose.pose.position != 0.0:
            self.altitude = pose.pose.position.z

    def update_twist(self, twist):
        """ if twist meets conditions fix, publish """
        # act on zoom input, ignore over an altitude
        if abs(twist.linear.z) > self.min_zoom and self.altitude < self.globe_view_altitude and self.tilt > 5:
            print(f"tilt: {self.tilt}, altitude: {self.altitude}, sqr alt: {math.sqrt(self.altitude)}")  # , twist in zoom: {twist} ")
            # tilt_varying_zoom = (twist.linear.z / 90) * max(((self.tilt + (self.tilt - 45)) + 15), self.tilt * 0.001)
            # "(zoom_in / 90) *" /*/ self.tilt=90 becomes => 195, 60=>105, 46=>63, 44=>57, 30=>15, 28=>9, 26=>3, 25=>0.025, 20=>0.02, 5=>0.005)
            tilt_varying_zoom = (twist.linear.z / 90) * max(((self.tilt + (2 * (self.tilt - 45))) + 15), self.tilt * 0.001)
            # factor in tilt on altitude multiplier
            tilt_altitude = self.altitude * self.tilt / 90
            twist.linear.x = -tilt_varying_zoom * tilt_altitude
            print(f'tvz: {tilt_varying_zoom}, ta: {tilt_altitude}, pan_x: {twist.linear.x}')

            # attempt to cancel out the zoom if over 90 tilt, could cause jumpiness?
            # starting higher zooms farther with same linear.z amount, could be a percentage?
            if self.tilt >= 90:
                twist.linear.z = -twist.linear.z
                print(f'canceling zoom {twist.linear.z}')
            # do not reset zoom if under 15 tilt, making it zoom faster to cancel out pan :)
            print("toma")
            if self.tilt > 15:
                twist.linear.z = 0.0000001
        print(f"twist out zoom_pan x: {twist.linear.x}")
        self.puber.publish(twist)


if __name__ == '__main__':
    MultiPanFix(min_zoom=0.00000001, min_pan=0.0001)
    rospy.spin()
