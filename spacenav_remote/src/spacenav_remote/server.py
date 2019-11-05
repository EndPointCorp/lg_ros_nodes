#!/usr/bin/env python3

import socketserver
import _thread
import socket


def print_handler(data):
    print(data)


class MyTCPHandler(socketserver.StreamRequestHandler):

    def __init__(self, callback, *args, **keys):
        self.callback = callback
        socketserver.StreamRequestHandler.__init__(self, *args, **keys)

    def handle(self):
        data = self.rfile.readline().strip()
        while data:
            self.callback(data)
            data = self.rfile.readline().strip()


def handler_factory(callback):
    def createHandler(*args, **keys):
        return MyTCPHandler(callback, *args, **keys)
    return createHandler


class SpacenavRemote(object):
    def __init__(self, handler=print_handler, port=6465):
        HOST, PORT = '', port

        # Create the server, binding to localhost on port 6564
        socketserver.TCPServer.allow_reuse_address = True
        self.server = socketserver.TCPServer((HOST, PORT), handler_factory(handler))
        self.server.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def fork_and_run(self):
        """
        Activate the server in separate thread
        """
        _thread.start_new_thread(self.serve_forever, ())

    def serve_forever(self):
        try:
            self.server.serve_forever()
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        self.server.shutdown()
        self.server.socket.close()


if __name__ == "__main__":
    print("Start spacenav remote server")
    server = SpacenavRemote()
    server.serve_forever()
