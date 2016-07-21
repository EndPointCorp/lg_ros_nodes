import sys
import rospy
import socket
import shutil
import shlex

from lg_common import ManagedApplication, ManagedWindow
from lg_common.tcp_relay import TCPRelay
from lg_common.msg import ApplicationState
from tornado.websocket import websocket_connect

DEFAULT_BINARY = '/usr/bin/google-chrome'
DEFAULT_ARGS = [
    '--enable-gpu-rasterization',
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
    '--v=1',
    '--enable-webgl',
    '--ignore-gpu-blacklist'
]


class ManagedBrowser(ManagedApplication):
    def __init__(self, url=None, slug=None, kiosk=True, geometry=None,
                 binary=DEFAULT_BINARY, remote_debugging_port=None, app=False,
                 shell=True, command_line_args='', disk_cache_size=314572800,
                 log_level=0, extensions=[], log_stderr=False, **kwargs):

        # If no slug provided, attempt to use the node name.
        if slug is None:
            try:
                slug = rospy.get_name().lstrip('/')
            except Exception as e:
                sys.stderr.write('Could not resolve slug for this browser!')
                sys.stderr.write(' * Has your node been initialized?')
                raise e

        cmd = [binary]

        # If no debug port provided, pick one.
        if remote_debugging_port is None:
            remote_debugging_port = ManagedBrowser.get_os_port()
        self.debug_port = ManagedBrowser.get_os_port()

        self.relay = TCPRelay(self.debug_port, remote_debugging_port)

        if log_stderr:
            cmd.append('--enable-logging=stderr')
        else:
            cmd.append('--enable-logging')
        cmd.append('--remote-debugging-port={}'.format(self.debug_port))
        cmd.append('--log-level={}'.format(log_level))

        self.tmp_dir = '/tmp/lg_browser_{}'.format(slug)
        self.clear_tmp_dir()

        cmd.append('--user-data-dir={}'.format(self.tmp_dir))
        cmd.append('--disk-cache-dir={}'.format(self.tmp_dir))
        cmd.append('--crash-dumps-dir={}/crashes'.format(self.tmp_dir))

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

        # finishing command line and piping output to logger
        cmd.extend(shlex.split('2>&1'))
        rospy.loginfo("Starting cmd: %s" % cmd)

        # Different versions of Chrome use different window instances.
        # This should match 'Google-chrome' as well as 'google-chrome'
        w_instance = 'oogle-chrome \\({}\\)'.format(self.tmp_dir)
        window = ManagedWindow(w_instance=w_instance, geometry=geometry)

        rospy.loginfo("Command {}".format(cmd))

        super(ManagedBrowser, self).__init__(cmd=cmd, window=window)

    def clear_tmp_dir(self):
        """
        Clears out all temporary files and disk cache for this instance.
        """
        try:
            rospy.loginfo("Purging ManagedBrowser directory: %s" % self.tmp_dir)
            shutil.rmtree(self.tmp_dir)
        except OSError, e:
            rospy.loginfo("Could not purge the %s directory because %s" % (self.tmp_dir, e))

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

    def _handle_respawn(self):
        """
        Clear tmp_dir upon respawn.
        """
        self.clear_tmp_dir()
        super(ManagedBrowser, self)._handle_respawn()

    def set_state(self, state):
        super(ManagedBrowser, self).set_state(state)

        if state == ApplicationState.STOPPED:
            self.relay.stop()

        elif state == ApplicationState.SUSPENDED:
            self.relay.start()

        elif state == ApplicationState.HIDDEN:
            self.relay.start()

        elif state == ApplicationState.VISIBLE:
            self.relay.start()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
