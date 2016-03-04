import sys
import rospy
import socket
import shutil

from lg_common import ManagedApplication, ManagedWindow
from tornado.websocket import websocket_connect

DEFAULT_BINARY = '/usr/bin/google-chrome'
DEFAULT_ARGS = [
    '--no-first-run',
    '--allow-file-access-from-files',
    '--disable-default-apps',
    '--disable-java',
    '--disable-session-storage',
    '--disable-translate',
    '--touch-events=enabled',
    '--disable-pinch',
    '--overscroll-history-navigation=0',
    '--disable-touch-editing',
    '--log-level=0',
    '--enable-logging',
    '--v=1',
]


class ManagedBrowser(ManagedApplication):
    def __init__(self, url=None, slug=None, kiosk=True, geometry=None,
                 binary=DEFAULT_BINARY, remote_debugging_port=None, app=False,
                 shell=True, command_line_args='', extensions=[], **kwargs):

        cmd = [binary]

        # If no debug port provided, pick one.
        if remote_debugging_port is None:
            remote_debugging_port = ManagedBrowser.get_os_port()
        self.debug_port = remote_debugging_port

        cmd.append('--remote-debugging-port={}'.format(self.debug_port))

        # If no slug provided, attempt to use the node name.
        if slug is None:
            try:
                slug = rospy.get_name().lstrip('/')
            except Exception as e:
                sys.stderr.write('Could not resolve slug for this browser!')
                sys.stderr.write(' * Has your node been initialized?')
                raise e

        tmp_dir = '/tmp/lg_browser-{}'.format(slug)
        try:
            rospy.loginfo("Purging ManagedBrowser directory: %s" % tmp_dir)
            shutil.rmtree(tmp_dir)
        except OSError, e:
            rospy.logerr("Could not purge the %s directory because %s" % (tmp_dir, e))

        cmd.append('--user-data-dir={}'.format(tmp_dir))
        cmd.append('--disk-cache-dir={}'.format(tmp_dir))
        cmd.append('--crash-dumps-dir={}/crashes'.format(tmp_dir))

        if extensions:
            cmd.append('--load-extension={}'.format(','.join(extensions)))

        cmd.extend(DEFAULT_ARGS)
        if command_line_args != '':
            cmd.extend(command_line_args)

        # All remaining kwargs are mapped to command line args.
        # _ is replaced with -.
        def consume_kwarg(item):
            key, value = item
            arg = '--{}'.format(key.replace('_', '-'))
            if value is None:
                return arg

            if isinstance(value, bool):
                arg += '=' + str(value).lower()
            else:
                arg += '=' + str(value)
            return arg

        args = map(consume_kwarg, kwargs.iteritems())
        cmd.extend(args)

        if app:
            cmd.append('--app={}'.format(url))
        else:
            if kiosk:
                cmd.append('--kiosk')
            if url is not None:
                cmd.append(url)

        cmd.append('&')
        # In Chrome 46 the instance changes from
        #   Google-chrome (...) to
        #   google-chrome (...)
        # Since all regex is escaped further down,
        # just don't match the 'g' for now.
        w_instance = 'oogle-chrome \\({}\\)'.format(tmp_dir)
        window = ManagedWindow(w_instance=w_instance, geometry=geometry)

        rospy.loginfo("Command {}".format(cmd))

        super(ManagedBrowser, self).__init__(cmd=cmd, window=window)

    @staticmethod
    def get_os_port():
        """
        Lets the OS assign a port number.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind(('', 0))
        port = sock.getsockname()[1]
        sock.close()
        return port

    def send_debug_sock_msg(self, msg):
        """
        Writes a string to the browser's debug web socket.
        """
        rospy.warn(
            'ManagedBrowser.send_debug_sock_msg() probably not yet working'
        )
        ws_url = 'ws://localhost:{}'.format(self.debug_port)
        conn = yield websocket_connect(ws_url, connect_timeout=1)
        conn.write_message(msg)
        conn.close()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
