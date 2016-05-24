from socket import *
import threading

import rospy
from geometry_msgs.msg import PoseStamped

MAX_DT = 0.02


class ViewsyncInterpolator:
    def __init__(self):
        self.position = [0.0 for i in range(3)]
        self.orientation = [0.0 for i in range(3)]
        self.v_position = [0.0 for i in range(3)]
        self.v_orientation = [0.0 for i in range(3)]

        self._t0 = None
        self._lock = threading.Lock()

    def get_time(self):
        return rospy.Time.now().to_sec()

    def _set_pose(self, position, orientation):
        t = self.get_time()
        t0 = self._t0 if self._t0 is not None else t
        self._t0 = t
        dt = min(max(t - t0, 0), MAX_DT)

        # bypass
        if dt != 0:
            self.v_position = [
                (m[0] - m[1]) / dt for m in zip(position, self.position)
            ]
            self.v_orientation = [
                (m[0] - m[1]) / dt for m in zip(orientation, self.orientation)
            ]
        else:
            self.v_position = [0.0 for i in range(3)]
            self.v_orientation = [0.0 for i in range(3)]

        self.position = position
        self.orientation = orientation

    def set_pose(self, position, orientation):
        with self._lock:
            return self._set_pose(position, orientation)

    def _get_pose(self):
        t = self.get_time()
        t0 = self._t0 if self._t0 is not None else t
        dt = min(max(t - t0, 0), MAX_DT)

        # bypass
        if dt == 0:
            return self.position, self.orientation

        position = [
            m[0] + m[1] * dt for m in zip(self.position, self.v_position)
        ]
        orientation = [
            m[0] + m[1] * dt for m in zip(self.orientation, self.v_orientation)
        ]

        return position, orientation

    def get_pose(self):
        with self._lock:
            return self._get_pose()


class ViewsyncSquawker:
    def __init__(self, repeat_addr, pose_pub, planet_pub):
        self.repeat_addr = repeat_addr
        self.pose_pub = pose_pub
        self.planet_pub = planet_pub

        self.repeat_sock = socket(AF_INET, SOCK_DGRAM)
        self.repeat_sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.repeat_sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

        self.counter = 1
        self.planet = ''
        self.interpolator = ViewsyncInterpolator()
        self.tick_timer = None
        self.tick_period = 1.0 / 240.0

    def _write_datagram(self, data):
        """Sends data to the repeat address.

        Args:
            data (str): Data to write to the repeat address.
        """
        self.repeat_sock.sendto(data, self.repeat_addr)

    def _publish_pose(self, pose_msg):
        """Publish a Pose.

        Args:
            pose_msg (PoseStamped): Pose to be published.
        """
        self.pose_pub.publish(pose_msg)

    def _publish_planet(self, planet):
        """Publish a Pose.

        Args:
            planet_msg (str): Planet name to be published.
        """
        self.planet_pub.publish(planet)

    def consume(self, position, orientation, planet):
        self.interpolator.set_pose(position, orientation)
        self.planet = planet

    def _tick(self, tev):
        position, orientation = self.interpolator.get_pose()
        planet = self.planet
        counter = self.counter

        # bypass: no pose received yet
        if all([m == 0 for m in position]):
            return

        self.counter += 1

        datagram = ','.join((
            str(counter),
            '{:f}'.format(position[0]),
            '{:f}'.format(position[1]),
            '{:f}'.format(position[2]),
            '{:f}'.format(orientation[2]),
            '{:f}'.format(orientation[0]),
            '{:f}'.format(orientation[1]),
            '-31574016',
            '-31574016',
            str(planet)
        ))
        self._write_datagram(datagram)

        msg = PoseStamped()
        msg.header.stamp = rospy.get_rostime()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        self._publish_pose(msg)

        self._publish_planet(planet)

    def start_timer(self):
        if self.tick_timer is not None:
            rospy.logwarn('Tried to start a running ViewsyncSquawker')
            return

        self.tick_timer = rospy.Timer(
            rospy.Duration.from_sec(self.tick_period),
            self._tick
        )


class ViewsyncRelay:
    def __init__(self, repeat_addr, pose_pub, planet_pub):
        """ViewSync sniffer and repeater.

        Publishes Earth's position as as Pose.

        Args:
            repeat_addr (str)
            pose_pub (rospy.Publisher)
            planet_pub (rospy.Publisher)
        """
        self.squawker = ViewsyncSquawker(repeat_addr, pose_pub, planet_pub)

        self.listen_sock = socket(AF_INET, SOCK_DGRAM)
        self.listen_sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

        self.listen_sock.bind(('', 0))
        self.listen_port = self.listen_sock.getsockname()[1]

    @staticmethod
    def parse_pose(data):
        """Turn an Earth ViewSync datagram into a Pose.

        Args:
            data (str): ViewSync datagram from Earth.

        Returns:
            position, orientation, planet
        """
        fields = data.split(',')
        position = [float(m) for m in fields[1:4]]
        # orientation comes in as heading, tilt, roll
        orientation = [float(fields[i]) for i in (5, 6, 4)]
        planet = str(fields[9])
        return position, orientation, planet

    def run(self):
        """Run the relay.

        This is a blocking method that runs until the ROS node is shutdown.
        """
        self.squawker.start_timer()

        while not rospy.is_shutdown():
            try:
                data = self.listen_sock.recv(255)
            except error as e:
                rospy.loginfo('socket interrupted, breaking loop')
                break

            position, orientation, planet = ViewsyncRelay.parse_pose(data)
            self.squawker.consume(position, orientation, planet)


# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
