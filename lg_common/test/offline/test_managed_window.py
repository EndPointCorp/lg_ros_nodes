import subprocess
import time
import unittest
import uuid

from lg_common import ManagedWindow
from lg_common.msg import WindowGeometry

GRACE_DELAY = 0.1  # How long to wait for window convergence, in seconds.


class TestManagedWindow(unittest.TestCase):
    def setUp(self):
        self.uid = str(uuid.uuid4())
        self.proc = subprocess.Popen(args=['xeyes', '-name', self.uid])

    def tearDown(self):
        self.proc.terminate()

    def _get_geometry(self):
        proc = subprocess.run(
            args="""xwininfo -name "{}" | awk '/\s*-geometry/ {{ print $2 }}'""".format(self.uid),
            shell=True,
            check=True,
            stdout=subprocess.PIPE,
        )
        return proc.stdout.decode('utf-8').strip()

    def _check_geometry(self, expected):
        raw_geometry = self._get_geometry()
        geometry = ManagedWindow.parse_geometry(raw_geometry)
        self.assertEqual(geometry.width, expected.width)
        self.assertEqual(geometry.height, expected.height)
        self.assertEqual(geometry.x, expected.x)
        self.assertEqual(geometry.y, expected.y)

    def _get_visible(self):
        proc = subprocess.run(
            args="""xwininfo -name "{}" -wm | grep -q 'Hidden'""".format(self.uid),
            shell=True,
            check=False,
        )
        return proc.returncode != 0

    def test_visible(self):
        window = ManagedWindow(
            w_name=self.uid,
            visible=False,
        )

        window.converge()
        time.sleep(GRACE_DELAY)

        self.assertFalse(self._get_visible())

        window.set_visibility(True)
        window.converge()
        time.sleep(GRACE_DELAY)

        self.assertTrue(self._get_visible())

    def test_geometry(self):
        # When testing in some desktop environments,
        # This geometry needs to not overlap any sidebars, or the test may fail.
        geometry = WindowGeometry(
            x=400,
            y=400,
            width=400,
            height=400,
        )
        window = ManagedWindow(
            w_name=self.uid,
            geometry=geometry,
            visible=True,
        )

        window.converge()
        time.sleep(GRACE_DELAY)

        self._check_geometry(geometry)

        geometry2 = WindowGeometry(
            x=500,
            y=400,
            width=400,
            height=400,
        )
        window.set_geometry(geometry2)
        window.converge()
        time.sleep(GRACE_DELAY)

        self._check_geometry(geometry2)
