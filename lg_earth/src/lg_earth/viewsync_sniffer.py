from socket import *

import rospy
from geometry_msgs.msg import PoseStamped


class ViewsyncSniffer:
    def run(self):
        rospy.init_node('lg_earth_sniffer')

        listen_host = rospy.get_param('~listen_host', '127.0.0.1')
        listen_port = rospy.get_param('~listen_port', 42000)
        repeat_host = rospy.get_param('~repeat_host', '<broadcast>')
        repeat_port = rospy.get_param('~repeat_port', 42000)

        pose_pub = rospy.Publisher(
            '/earth/pose', PoseStamped, queue_size=3
        )

        listen_sock = socket(AF_INET, SOCK_DGRAM)
        listen_sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        listen_sock.bind((listen_host, listen_port))

        repeat_sock = socket(AF_INET, SOCK_DGRAM)
        repeat_sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        repeat_sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

        while not rospy.is_shutdown():
            try:
                data = listen_sock.recv(512)
            except error as e:
                rospy.loginfo('socket interrupted, breaking loop')
                break
            repeat_sock.sendto(data, (repeat_host, repeat_port))

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
            pose_pub.publish(msg)


# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
