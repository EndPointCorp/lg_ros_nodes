import os
import shutil
import threading
import xml.etree.ElementTree as ET
from xml.dom import minidom
from tempfile import gettempdir as systmp

import rospy
from lg_common.msg import ProcessState, WindowGeometry
from lg_common import ManagedProcess, ManagedWindow
from client_config import ClientConfig

TOOLBAR_HEIGHT = 22


class Client:
    def __init__(self):
        x = rospy.get_param('~window_x', 0)
        y = rospy.get_param('~window_y', 0) - TOOLBAR_HEIGHT
        w = rospy.get_param('~window_w', 640)
        h = rospy.get_param('~window_h', 480) + TOOLBAR_HEIGHT
        geometry = WindowGeometry(x=x, y=y, width=w, height=h)
        earth_window = ManagedWindow(
            geometry=geometry,
            w_instance=self._get_instance()
        )

        cmd = ['/opt/google/earth/free/googleearth-bin']
        args, geplus_config, layers_config, kml_content, view_content = self._get_config()
        cmd.extend(args)
        self.earth_proc = ManagedProcess(cmd, window=earth_window, suspend=True)

        self._make_dir()

        os.mkdir(self._get_dir() + '/.googleearth')
        os.mkdir(self._get_dir() + '/.googleearth/Cache')
        os.mkdir(self._get_dir() + '/.config')
        os.mkdir(self._get_dir() + '/.config/Google')
        self._render_config(geplus_config, '.config/Google/GoogleEarthPlus.conf')
        self._render_config(layers_config, '.config/Google/GECommonSettings.conf')
        self._render_file(kml_content, '.googleearth/myplaces.kml')
        self._render_file(view_content, '.googleearth/cached_default_view.kml')

        os.environ['HOME'] = self._get_dir()

        os.environ['BROWSER'] = '/dev/null'

        if os.getenv('DISPLAY') is None:
            os.environ['DISPLAY'] = ':0'

        os.environ['LD_LIBRARY_PATH'] += ':/opt/google/earth/free'

    def _get_instance(self):
        return '_earth_instance_' + rospy.get_name().strip('/')

    def _get_dir(self):
        return os.path.normpath(systmp() + '/' + self._get_instance())

    def _make_dir(self):
        self._clean_dir()
        os.mkdir(self._get_dir())
        assert os.path.exists(self._get_dir())
        rospy.on_shutdown(self._clean_dir)

    def _clean_dir(self):
        try:
            shutil.rmtree(self._get_dir())
        except OSError:
            pass

    def _get_config(self):
        config = ClientConfig(self._get_dir(), self._get_instance())
        return config.get_config()

    def _render_file(self, content, path):
        with open(self._get_dir() + '/' + path, 'w') as f:
            f.write(content)

    def _render_config(self, config, path):
        with open(self._get_dir() + '/' + path, 'w') as f:
            for section, settings in config.iteritems():
                f.write('[' + section + ']\n')
                for k, v in settings.iteritems():
                    r = str(v).lower() if isinstance(v, bool) else str(v)
                    f.write(k + '=' + r + '\n')
                f.write('\n')

    def run(self):
        self.earth_proc.set_state(ProcessState.ACTIVE)

        # temp/test activation code
        def handle_state_msg(msg):
            self.earth_proc.set_state(msg.state)

        rospy.Subscriber('/earth/state', ProcessState, handle_state_msg)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
