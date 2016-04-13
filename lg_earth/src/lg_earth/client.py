import os
import shutil
import threading
import xml.etree.ElementTree as ET
from xml.dom import minidom
from tempfile import gettempdir as systmp

import rospy
from lg_common.msg import ApplicationState, WindowGeometry
from lg_common import ManagedApplication, ManagedWindow
from client_config import ClientConfig

TOOLBAR_HEIGHT = 22

CUSTOM_CONFIG_DIR = '/lg'


class Client:
    """Google Earth client launcher."""
    def __init__(self, initial_state=None):
        args, geplus_config, layers_config, kml_content, view_content = \
            self._get_config()

        # Skip window management if this window is hidden.
        if '--hidegui' in args:
            earth_window = None
        else:
            geometry = ManagedWindow.get_viewport_geometry()
            if geometry is not None:
                geometry.y -= TOOLBAR_HEIGHT
                geometry.height += TOOLBAR_HEIGHT

            earth_window = ManagedWindow(
                geometry=geometry,
                w_class='Googleearth-bin',
                w_name='Google Earth',
                w_instance=self._get_instance()
            )

        cmd = ['/opt/google/earth/free/googleearth-bin']

        cmd.extend(args)
        self.earth_proc = ManagedApplication(cmd, window=earth_window,
                                             initial_state=initial_state)

        self._make_tempdir()

        os.mkdir(self._get_tempdir() + '/.googleearth')
        os.mkdir(self._get_tempdir() + '/.googleearth/Cache')

        if not rospy.get_param('~show_google_logo', True):
            source = '/home/lg/etc/localdbrootproto'
            dest = '/home/lg/.googleearth/Cache/localdbrootproto'
            if not os.path.exists(os.path.dirname(os.path.dirname(dest))):
                os.mkdir(os.path.dirname(os.path.dirname(dest)))
            if not os.path.exists(os.path.dirname(dest)):
                os.mkdir(os.path.dirname(dest))
            self._touch_file(source)
            with open(source, 'r') as src:
                with open(dest, 'w') as dst:
                    for line in src.readlines():
                        dst.write(line)

        os.mkdir(self._get_tempdir() + '/.config')
        os.mkdir(self._get_tempdir() + '/.config/Google')

        self._render_config(geplus_config,
                            '.config/Google/GoogleEarthPlus.conf')
        self._render_config(layers_config,
                            '.config/Google/GECommonSettings.conf')
        self._render_file(kml_content,
                          '.googleearth/myplaces.kml')
        self._render_file(view_content,
                          '.googleearth/cached_default_view.kml')

        # Check whether a non-standard GECommonSettings file exists
        # and replace if so
        #self._check_for_custom_config('./config/Google/GECommonSettings.conf')

        # Override the HOME location. This requires a hack to the Earth
        # libraries. See the lg_earth README.
        os.environ['OLDHOME'] = os.environ['HOME']
        os.environ['HOME'] = self._get_tempdir()

        # Prevent external browser launch.
        os.environ['BROWSER'] = '/dev/null'

        # If the Xorg DISPLAY is not in our environment, assume :0
        if os.getenv('DISPLAY') is None:
            os.environ['DISPLAY'] = ':0'

        # Google Earth has its own copies of some libraries normally found on
        # the system, so we need to tell the loader to look there. This is
        # normally done in the google-earth wrapper script.
        os.environ['LD_LIBRARY_PATH'] += ':/opt/google/earth/free'

    def _touch_file(self, fname):
        """Touch a file, updating its access time.

        Args:
            fname (str): Absolute path of the file to be touched.
        """
        if os.path.exists(fname):
            os.utime(fname, None)
        else:
            open(fname, 'a').close()

    def _get_instance(self):
        """Get a unique instance name for this client.

        Returns:
            str: This node's ROS name, sanitized for use as a temp path and
                window identifier.
        """
        return '_earth_instance_' + rospy.get_name().strip('/')

    def _get_tempdir(self):
        """Get this client's unique temporary path.

        Returns:
            str: Path to this client's temporary directory.
        """
        return os.path.normpath(systmp() + '/' + self._get_instance())

    def _make_tempdir(self):
        """Create a temporary directory and register cleanup on shutdown."""
        self._clean_tempdir()
        os.mkdir(self._get_tempdir())
        assert os.path.exists(self._get_tempdir())
        rospy.on_shutdown(self._clean_tempdir)

    def _clean_tempdir(self):
        """Attempt to delete temporary directory."""
        try:
            shutil.rmtree(self._get_tempdir())
        except OSError:
            pass

    def _get_config(self):
        """Grab configuration from the ClientConfig helper.

        Returns:
            tuple: See ClientConfig.get_config().
        """
        config = ClientConfig(self._get_tempdir(), self._get_instance())
        return config.get_config()

    def _render_file(self, content, path):
        """Checks if a custom file is provided, if it is use it, if not
        write standard content to that file.

        Args:
            content (str)
            path (str): File path relative to the temporary directory.
        """
        if self._check_for_custom_config(path):
            self._use_custom_config(path)
            return

        with open(self._get_tempdir() + '/' + path, 'w') as f:
            f.write(content)

    def _render_config(self, config, path):
        """Render a configuration file.

        This method first checks for a custom config, if present copy it into
        place. If not it takes a dictionary and writes the values as key value
        pairs. Boolean values are written lowercase.

        Args:
            config (Dict[str, object]): Key value pairs to write to the config
                file.
            path (str): Config file path relative to the temp directory.
        """
        if self._check_for_custom_config(path):
            self._use_custom_config(path)
            return

        with open(self._get_tempdir() + '/' + path, 'w') as f:
            for section, settings in config.iteritems():
                f.write('[' + section + ']\n')
                for k, v in settings.iteritems():
                    r = str(v).lower() if isinstance(v, bool) else str(v)
                    f.write(k + '=' + r + '\n')
                f.write('\n')

    def _check_for_custom_config(self, standard_conf_path):
        """Checks for a custom supplied config file and if present returns
        true.

        Args:
            standard_conf_path (str): Path of the standard config
        """

        ret_val = False
        conf_filename = os.path.basename(standard_conf_path)
        custom_conf_expected_path = CUSTOM_CONFIG_DIR + '/' + conf_filename

        if os.path.isfile(custom_conf_expected_path):
            ret_val = True

        return ret_val

    def _use_custom_config(self, standard_conf_path):
        """Moves the custom config file to the standard config path

        Args:
            standard_conf_path (str): Path of the standard config
        """
        conf_filename = os.path.basename(standard_conf_path)
        custom_conf_expected_path = CUSTOM_CONFIG_DIR + '/' + conf_filename
        shutil.copy(custom_conf_expected_path,
                    self._get_tempdir() + '/' + standard_conf_path)

    def _handle_soft_relaunch(self, msg):
        """
        Clearing up logs is pretty important for soft relaunches
        """
        rospy.logerr('removing cache for google earth')
        try:
            earth_dir = '%s/.googleearth' % os.environ['OLDHOME']
            shutil.rmtree(earth_dir)
            os.mkdir(earth_dir)
        except Exception, e:
            rospy.logerr('found error: %s' % e.message)
        self.earth_proc.handle_soft_relaunch()

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
