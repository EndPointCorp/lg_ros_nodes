#!/usr/bin/env python

"""
test_onboard - unit tests for onboard.py

offline tests - not requiring the ROS node itself to run.

"""


import os

import pytest
import rospkg

from lg_keyboard import OnboardConfig


class TestOnboardConfig(object):

    def test_onboard_config_basic(self):
        """
        Walks over everything in the class ...

        """
        config = OnboardConfig()
        assert "docking-enabled=false" in config.get_config()


if __name__ == "__main__":
    test_pkg = "lg_keyboard"
    test_name = "test_onboard"
    test_dir = os.path.join(rospkg.get_test_results_dir(env=None), test_pkg)
    pytest_result_path = os.path.join(test_dir, "rosunit-%s.xml" % test_name)
    # run only itself
    test_path = os.path.abspath(os.path.abspath(__file__))
    pytest.main("-vv -rfsX -s --junit-xml=%s %s" % (pytest_result_path, test_path))
