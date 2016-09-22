#!/usr/bin/env python

"""
lg_onboard basic offline tests (not requiring the ROS node itself to run).

"""


import os

import pytest
import rospkg

from lg_keyboard import OnboardManager


class TestOnboardManager(object):

    def test_something_basic(self):
        om = OnboardManager()
        assert True


if __name__ == "__main__":
    test_pkg = "lg_keyboard"
    test_name = "test_basic"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # run only itself
    test_path = os.path.abspath(os.path.abspath(__file__))
    # output is unfortunately handled / controlled by above layer of rostest (-s has no effect)
    #pytest.main("-vv -rfsX -s --junit-xml=%s --cov=. --cov-report term-missing %s" % (pytest_result_path, test_path))
    pytest.main("-vv -rfsX -s --junit-xml=%s %s" % (pytest_result_path, test_path))
