from socket import *

import rospy
from geometry_msgs.msg import PoseStamped


class ViewsyncRelay:
    def __init__(self, repeat_addr, pose_pub, planet_pub):
        """ViewSync sniffer and repeater.

        Publishes Earth's position as as Pose.

        Args:
            repeat_addr (str)
            pose_pub (rospy.Publisher)
        """
        self.repeat_addr = repeat_addr
        self.pose_pub = pose_pub
        self.planet_pub = planet_pub

        self.listen_sock = socket(AF_INET, SOCK_DGRAM)
        self.listen_sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

        self.repeat_sock = socket(AF_INET, SOCK_DGRAM)
        self.repeat_sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.repeat_sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

        self.listen_sock.bind(('', 0))
        self.listen_port = self.listen_sock.getsockname()[1]


    @staticmethod
    def parse_pose(data):
        """Turn an Earth ViewSync datagram into a Pose.

        Args:
            data (str): ViewSync datagram from Earth.

        Returns:
            PoseStamped: Camera position.
        """
        fields = data.split(',')
        msg = PoseStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.pose.position.x = float(fields[1])
        msg.pose.position.y = float(fields[2])
        msg.pose.position.z = float(fields[3])
        msg.pose.orientation.z = float(fields[4])
        msg.pose.orientation.x = float(fields[5])
        msg.pose.orientation.y = float(fields[6])
        msg.pose.orientation.w = 0
        return [msg, str(fields[9])]

    def _repeat(self, data):
        """Sends data to the repeat address.

        Args:
            data (str): Data to write to the repeat address.
        """
        self.repeat_sock.sendto(data, self.repeat_addr)

    def _publish_pose_msg(self, pose_msg, planet):
        """Publish a Pose.

        Args:
            pose_msgs (PoseStamped): Pose to be published.
        """
        self.pose_pub.publish(pose_msg)
        self.planet_pub.publish(planet)

    def run(self):
        """Run the relay.

        This is a blocking method that runs until the ROS node is shutdown.
        """
        while not rospy.is_shutdown():
            try:
                data = self.listen_sock.recv(255)
            except error as e:
                rospy.loginfo('socket interrupted, breaking loop')
                break

            self._repeat(data)

            pose_msg, planet = ViewsyncRelay.parse_pose(data)
            self._publish_pose_msg(pose_msg, planet)


# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
