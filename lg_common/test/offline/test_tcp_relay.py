#!/usr/bin/env python3

PKG = 'lg_common'
NAME = 'test_tcp_relay'

import rospy
import unittest
import socket
import threading
import time

from lg_common.tcp_relay import TCPRelay

MAGIC = b'MAGIC\n'


def get_ephemeral_port():
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('', 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


class Server:
    def __init__(self, port):
        self.port = port
        self.messages = []
        self._shutdown = False

    def listen(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind(('127.0.0.1', self.port))
        sock.listen(1)
        conn, addr = sock.accept()
        while not self._shutdown:
            msg = conn.recv(1024)
            if not msg:
                break
            self.messages.append(msg)
            conn.sendall(msg)
        sock.close()

    def shutdown(self):
        self._shutdown = True


class TestTCPRelay(unittest.TestCase):
    def test_tcp_relay(self):
        """Test bi-directional communication over the relay."""
        self.local_port = get_ephemeral_port()
        self.remote_port = get_ephemeral_port()

        self.server = Server(self.local_port)
        self.server_thread = threading.Thread(target=self.server.listen)
        self.server_thread.daemon = True
        self.server_thread.start()

        self.relay = TCPRelay(self.local_port, self.remote_port)
        self.relay.start()

        # grace delay
        time.sleep(1.0)

        remote_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        remote_sock.settimeout(1.0)
        remote_sock.connect(('127.0.0.1', self.remote_port))

        remote_sock.sendall(MAGIC)
        response = remote_sock.recv(1024)

        self.assertEqual(MAGIC, response)
        self.assertEqual(1, len(self.server.messages))
        self.assertEqual(MAGIC, self.server.messages[0])

        remote_sock.close()


if __name__ == '__main__':
    import rostest
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestTCPRelay)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
