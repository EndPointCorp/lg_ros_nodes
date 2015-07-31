"""
Ad hoc media application manager tests.

"""

import subprocess
import tempfile
import time

import pytest
import rostest


class TestMediaService(object):

    roslaunch = None  # roslaunch process
    stdout, stderr = None, None

    @classmethod
    def setup_class(cls):
        cmd = "roslaunch lg_media/launch/dev.launch --screen"
        print "Starting roslaunch ('%s') ..." % cmd
        cls.stdout, cls.stderr = tempfile.TemporaryFile("w+"), tempfile.TemporaryFile("w+")
        cls.roslaunch = subprocess.Popen(cmd.split(),
                                         stdout=cls.stdout,
                                         stderr=cls.stderr,
                                         close_fds=False)
        time.sleep(2)
        assert cls.roslaunch.poll() is None
        print dir(cls.roslaunch)
        # TODO
        # make some inteligent assert on services, topics available

    @classmethod
    def teardown_class(cls):
        assert cls.roslaunch.poll() is None
        cls.roslaunch.kill()
        time.sleep(1)
        assert cls.roslaunch.poll() is not None
        cls.stdout.seek(0)
        cls.stderr.seek(0)
        print "roslaunch output:"
        print "stdout: --------------------------\n"
        print cls.stdout.read()
        print "stderr: --------------------------\n"
        print cls.stderr.read()

    def setup_method(self, method):
        print "setup_method, running '%s' ..." % method.__name__

    def teardown_method(self, _):
        print "teardown_method"

    def test_1(self):
        print "test_1"

    def test_2(self):
        print "test_2"


if __name__ == "__main__":
    rostest.rosrun("lg_media", "test_lg_media_basic", TestMediaService)