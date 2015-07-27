from socket import *

import rospy
from geometry_msgs.msg import PoseStamped


class ViewsyncRelay:
    def __init__(self, listen_addr, repeat_addr, pose_pub):
        """ViewSync sniffer and repeater.

        Publishes Earth's position as as Pose.

        Args:
            listen_addr (str)
            repeat_addr (str)
            pose_pub (rospy.Publisher)
        """
        self.listen_addr = listen_addr
        self.repeat_addr = repeat_addr
        self.pose_pub = pose_pub

        self.listen_sock = socket(AF_INET, SOCK_DGRAM)
        self.listen_sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

        self.repeat_sock = socket(AF_INET, SOCK_DGRAM)
        self.repeat_sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.repeat_sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

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
        return msg

    def _repeat(self, data):
        """Sends data to the repeat address.

        Args:
            data (str): Data to write to the repeat address.
        """
        self.repeat_sock.sendto(data, self.repeat_addr)

    def _publish_pose_msg(self, pose_msg):
        """Publish a Pose.

        Args:
            pose_msgs (PoseStamped): Pose to be published.
        """
        self.pose_pub.publish(pose_msg)

    def run(self):
        """Run the relay.

        This is a blocking method that runs until the ROS node is shutdown.
        """
        self.listen_sock.bind(self.listen_addr)

        while not rospy.is_shutdown():
            try:
                data = self.listen_sock.recv(255)
            except error as e:
                rospy.loginfo('socket interrupted, breaking loop')
                break

            self._repeat(data)

            pose_msg = ViewsyncRelay.parse_pose(data)
            self._publish_pose_msg(pose_msg)


# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
