#!/usr/bin/env python

import SocketServer

def print_handler(data):
    print data

class MyTCPHandler(SocketServer.StreamRequestHandler):

    def __init__(self, callback, *args, **keys):
        self.callback = callback
        SocketServer.StreamRequestHandler.__init__(self, *args, **keys)

    def handle(self):
        # self.rfile is a file-like object created by the handler;
        # we can now use e.g. readline() instead of raw recv() calls
        self.data = self.request.recv(1024).strip()
        self.callback(self.data)

def handler_factory(callback):
    def createHandler(*args, **keys):
        return MyTCPHandler(callback, *args, **keys)
    return createHandler

class SpacenavRemote(object):
    def __init__(self, handler=print_handler, port=6465):
        HOST, PORT = '', port

        # Create the server, binding to localhost on port 6564
        SocketServer.TCPServer.allow_reuse_address = True
        self.server = SocketServer.TCPServer((HOST, PORT), handler_factory(handler))

        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        try:
            self.server.serve_forever()
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        self.server.shutdown()
        self.server.socket.close()
