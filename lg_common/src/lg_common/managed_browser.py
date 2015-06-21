import sys

import rospy

from lg_common import ManagedApplication, ManagedWindow

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
                 binary=DEFAULT_BINARY, **kwargs):

        cmd = [binary]

        # If no slug provided, attempt to use the node name.
        if slug is None:
            try:
                slug = rospy.get_name().lstrip('/')
            except Exception as e:
                sys.stderr.write('Could not resolve slug for this browser!')
                sys.stderr.write(' * Has your node been initialized?')
                raise e

        tmp_dir = '/tmp/lg_browser-{}'.format(slug)
        cmd.append('--user-data-dir={}'.format(tmp_dir))
        cmd.append('--disk-cache-dir={}'.format(tmp_dir))
        cmd.append('--crash-dumps-dir={}/crashes'.format(tmp_dir))

        cmd.extend(DEFAULT_ARGS)

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

        if kiosk:
            cmd.append('--kiosk')
        if url is not None:
            cmd.append(url)

        w_instance = 'Google-chrome \\({}\\)'.format(tmp_dir)
        window = ManagedWindow(w_instance=w_instance, geometry=geometry)

        super(ManagedBrowser, self).__init__(cmd=cmd, window=window)

# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
