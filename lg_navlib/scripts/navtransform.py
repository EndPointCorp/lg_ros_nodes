mport math
import rospy
from geometry_msgs.msg import Twist, PoseStamped


class NavTransform(object):
    """ transform twist messages """
    def __init__(self):
        self.node = rospy.init_node('navtransform')
        self.puber = rospy.Publisher('/navtransform/twist', Twist, queue_size=10)
        self.sub_twist = rospy.Subscriber("/lg_navlib/twist", Twist, self.transform_twist)
        self.sub_pose = rospy.Subscriber("/earth/pose", PoseStamped, self.update_pose)
        self.tilt = 0.0
        self.altitude = 0.0

    def update_pose(self, pose):
        """ update tilt, other values from pose """
        self.tilt = pose.pose.orientation.x
        self.altitude = pose.pose.position.z

    def transform_twist(self, twist):
        """ fix twist, publish """
        # convert zoom into panned-zoom
        down = min(max(twist.linear.z, -1), 1) * -1
        tiltness = min(1.0, max(0.0, self.tilt / 90))
        if twist.linear.z != 0:
            twist.linear.x = twist.linear.x * (1 - tiltness) + down * tiltness
            twist.linear.z *= 1.0 - tiltness

#       # limit tilt up to 50-55
        if twist.angular.y < 0:
            if self.tilt > 55:
                twist.angular.y = 0.05
            elif self.tilt >= 50:
                twist.angular.y = 0.0
#           # add pan when tilting
            else:
                twist.linear.x = twist.angular.y * self.tilt / 90
        if twist.angular.y > 0 and self.tilt > 0:
            twist.linear.x = twist.angular.y * self.tilt / 90

#       # correct tilt when zooming out to keep earth on screen
        if twist.linear.z > 0 and self.altitude > 1000000 and self.tilt > 0:
            twist.angular.y = 0.4

#       # correct tilt when zooming in close to the ground
        if twist.linear.z < 0 and self.altitude < 10000 and self.tilt < 45:
            twist.angular.y = twist.linear.z

        self.puber.publish(twist)


if __name__ == '__main__':
    NavTransform()
    rospy.spin()

