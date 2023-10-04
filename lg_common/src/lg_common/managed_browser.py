import sys
import rospy
import socket
import shutil
import os
import requests
import threading

from lg_common import ManagedApplication, ManagedWindow
from lg_common.tcp_relay import TCPRelay
from lg_msg_defs.msg import ApplicationState
from tornado.websocket import websocket_connect

from lg_common.logger import get_logger
logger = get_logger('managed_browser')

DEFAULT_BINARY = '/usr/bin/google-chrome'
DEFAULT_ARGS = [
    '--no-first-run',
    '--no-sandbox',
    '--test-type',  # only needed to ignore --no-sandbox's warning message
    '--allow-file-access-from-files',
    '--disable-default-apps',
    '--disable-java',
    '--disable-session-storage',
    '--disable-translate',
    '--touch-events=enabled',
    '--disable-pinch',
    '--overscroll-history-navigation=0',
    '--allow-running-insecure-content',
    '--disable-touch-editing',
    '--v=1',
    '--touch-events=enabled',
    '--disable-pinch',
    '--overscroll-history-navigation=0',
    '--autoplay-policy=no-user-gesture-required',
    '--check-for-update-interval=1209600',
    '--ignore-gpu-blacklist',
    '--ignore-gpu-blocklist',
    '--enable-gpu-rasterization',
    '--simulate-outdated-no-au=\'Tue, 31 Dec 2099 23:59:59 GMT\'',
    '--enable-features=VaapiVideoDecoder,VaapiVideoEncoder,CanvasOopRasterization',
    '--disable-gpu-driver-bug-workarounds',
    '--disable-features=UseChromeOSDirectVideoDecoder',
]


def set_interval(func, sec):
    def func_wrapper():
        set_interval(func, sec)
        func()
    t = threading.Timer(sec, func_wrapper)
    t.start()
    return t


class ManagedBrowser(ManagedApplication):
    def __init__(
        self,
        url=None,
        slug=None,
        kiosk=True,
        user_data_dir=None,
        geometry=None,
        binary=DEFAULT_BINARY,
        remote_debugging_port=None,
        app=False,
        reload_aw_snap=False,
        shell=True,
        command_line_args=[],
        default_args_removal=[],
        disk_cache_size=314572800,
        log_level=0,
        extensions=[],
        log_stderr=False,
        user_agent='',
        pepper_flash_dir='/home/lg/inc/PepperFlash',
        pnacl_dir='/home/lg/inc/pnacl',
        layer=ManagedWindow.LAYER_NORMAL,
        **kwargs
    ):

        # If no slug provided, attempt to use the node name.
        if slug is None:
            try:
                slug = rospy.get_name().lstrip('/')
            except Exception as e:
                sys.stderr.write('Could not resolve slug for this browser!')
                sys.stderr.write(' * Has your node been initialized?')
                raise e

        cmd = [binary]

        if user_agent:
            cmd.append('--user-agent={}'.format(user_agent))

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

        self.user_data_dir = user_data_dir

        if self.user_data_dir:
            logger.debug('using data dir {}'.format(self.user_data_dir))
            self.tmp_dir = '/tmp/user_data_dirs/{}'.format(self.user_data_dir)
        else:
            self.tmp_dir = '/tmp/lg_browser_{}'.format(slug)
            logger.debug('clearing tmp dir {}'.format(self.tmp_dir))
        self.clear_tmp_dir()
        self.pepper_flash_dir = pepper_flash_dir
        self.pnacl_dir = pnacl_dir
        self.init_tmp_dir()

        cmd.append('--user-data-dir={}'.format(self.tmp_dir))
        cmd.append('--disk-cache-dir={}'.format(self.tmp_dir))
        cmd.append('--crash-dumps-dir={}/crashes'.format(self.tmp_dir))

        if extensions:
            extensions = extensions.split()
            for extension in extensions:
                if not os.path.isdir(extension):
                    extensions.remove(extension)
                    logger.warning("Could not load extension from %s because dir does not exist" % extension)
            if extensions:
                cmd.append('--load-extension={}'.format(','.join(extensions)))

        for _cmd in default_args_removal:
            if _cmd in DEFAULT_ARGS:
                DEFAULT_ARGS.remove(_cmd)

        cmd.extend(DEFAULT_ARGS)
        if command_line_args != []:
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

        args = list(map(consume_kwarg, iter(kwargs.items())))
        cmd.extend(args)

        if app:
            cmd.append('--app={}'.format(url))
        else:
            if kiosk:
                cmd.append('--kiosk')
                pass
            if url is not None:
                cmd.append(url)

        # finishing command line and piping output to logger
        logger.debug("Starting cmd: %s" % cmd)

        # Different versions of Chrome use different window instance names.
        # Matching the tmp_dir should work for all of them.
        w_instance = 'google-chrome ({})'.format(self.tmp_dir)
        window = ManagedWindow(
            w_instance=w_instance,
            geometry=geometry,
            chrome_kiosk_workaround=kiosk,
            layer=layer,
        )

        logger.debug("Command {}".format(cmd))

        if (reload_aw_snap):
            self.set_aw_snap_timer()

        # clean up after thyself
        rospy.on_shutdown(self.clear_tmp_dir)

        super(ManagedBrowser, self).__init__(cmd=cmd, window=window, stdout=open('/{}/browser.log'.format(self.tmp_dir), 'w'), stderr=open('/{}/browser_error.log'.format(self.tmp_dir), 'w'))

    def post_init(self):
        super(ManagedBrowser, self).post_init()

        if not self.user_data_dir:
            self.add_respawn_handler(self.clear_tmp_dir)
        self.add_respawn_handler(self.init_tmp_dir)
        self.add_state_handler(self.control_relay)

    def init_tmp_dir(self):
        """
        Creates the tmp dir
        then links in the path to Chrome components like PepperFlash
        then replaces the path in the latest-copmponent-updated-flash file
        """

        if os.path.exists(self.tmp_dir):
            if self.user_data_dir:
                return  # this is fine
            else:
                logger.error("Temp dir exists for chrome already")
        try:
            os.mkdir(self.tmp_dir)
            os.mkdir(self.tmp_dir + '/PepperFlash')
        except Exception:
            logger.error("Error trying to make the tmp dir, could exist already")

        # Link NaCl component. https://github.com/EndPointCorp/lg_ros_nodes/issues/357
        try:
            os.symlink(self.pnacl_dir, os.path.join(self.tmp_dir, 'pnacl'))
            logger.info("Linked `pnacl` directory %s" % self.pnacl_dir)
        except Exception as e:
            logger.error("Error linking pNaCl, %s" % e)

        try:
            os.symlink(self.pepper_flash_dir + '/flash_dir', "%s/PepperFlash/flash_dir" % self.tmp_dir)
            with open("%s/latest-component-updated-flash" % self.pepper_flash_dir, "r") as f:
                out = f.read()
            with open("%s/PepperFlash/latest-component-updated-flash" % self.tmp_dir, "w") as f:
                f.write(out.replace("${TMP_DIR}", self.tmp_dir))
        except Exception as e:
            logger.error("Error copying pepper flash into the tmp dir, %s" % e)

    def clear_tmp_dir(self):
        """
        Clears out all temporary files and disk cache for this instance.
        """
        if self.user_data_dir:
            logger.error('not clearing, because user data dir')
            return
        try:
            logger.error("Purging ManagedBrowser directory: %s" % self.tmp_dir)
            shutil.rmtree(self.tmp_dir)
        except OSError as e:
            logger.debug("Could not purge the %s directory because %s" % (self.tmp_dir, e))

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

    def list_pages_available_for_debug(self):
        debug_url = 'http://localhost:{}/json/list'.format(self.debug_port)
        return requests.get(debug_url).json()

    def set_aw_snap_timer(self):
        self.aw_snap_interval = set_interval(self.check_alive_and_reload, 1)

    def check_alive_and_reload(self):
        if not self.check_alive():
            logger.error("Browser is probably dead")
            self.reload_page()

    def reload_page(self):
        pid = self.proc.get_pid()
        if pid:
            cmd = "DISPLAY=:0 xdotool search --onlyvisible --all --pid {} --class Chrome windowfocus key F5".format(pid)
            os.system(cmd)

    def check_alive(self):
        return len(self.list_pages_available_for_debug()) > 0

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

    def control_relay(self, state):
        if state == ApplicationState.STOPPED:
            self.relay.stop()

        elif state == ApplicationState.SUSPENDED:
            self.relay.start()

        elif state == ApplicationState.HIDDEN:
            self.relay.start()

        elif state == ApplicationState.VISIBLE:
            self.relay.start()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
