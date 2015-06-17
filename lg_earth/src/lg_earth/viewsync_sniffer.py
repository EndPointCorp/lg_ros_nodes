import SocketServer
import threading

import rospy
from geometry_msgs.msg import PoseStamped


def _get_publisher():
    if not hasattr(_get_publisher, 'instance'):
        _get_publisher.instance = rospy.Publisher(
            '/earth/pose', PoseStamped, queue_size=3
        )
    return _get_publisher.instance


def _get_repeat_target():
    if not hasattr(_get_repeat_target, 'target'):
        host = rospy.get_param('~repeat_host', '127.0.0.255')
        port = rospy.get_param('~repeat_port', 42000)
        _get_repeat_target.target = (host, port)
    return _get_repeat_target.target


class ViewsyncHandler(SocketServer.BaseRequestHandler):
    def setup(self):
        self.pub = _get_publisher()
        self.target = _get_repeat_target()

    def handle(self):
        data = self.request[0]
        socket = self.request[1]
        socket.sendto(data, self.target)
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
        self.pub.publish(msg)


class ThreadedUDPServer(SocketServer.ThreadingMixIn, SocketServer.UDPServer):
    pass


class ViewsyncSniffer:
    def run(self):
        rospy.init_node('lg_earth_sniffer')

        self.listen_host = rospy.get_param('~listen_host', '127.0.0.1')
        self.port = rospy.get_param('~listen_port', 42000)

        ThreadedUDPServer.allow_reuse_address = True
        server = ThreadedUDPServer((self.listen_host, self.port), ViewsyncHandler)
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.daemon = True
        server_thread.start()

        rospy.on_shutdown(server.shutdown)

        rospy.spin()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
